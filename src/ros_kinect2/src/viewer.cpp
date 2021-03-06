/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

class Receiver
{
public:
  enum Mode { IMAGE = 0, CLOUD, BOTH };

private:
    std::mutex lock;

    const std::string topicColor, topicDepth;
    const bool useExact, useCompressed;

    bool updateImage, updateCloud;
    bool save;
    bool running;
    size_t frame;
    const size_t queueSize;

    cv::Mat color, depth;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

    std::thread imageViewerThread;
    Mode mode;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PCDWriter writer;
    std::ostringstream oss;
    std::vector<int> params;

public:
    Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
    updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
    nh("~"), spinner(0), it(nh), mode(CLOUD) {
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(100);
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(1);
        params.push_back(cv::IMWRITE_PNG_STRATEGY);
        params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
        params.push_back(0);
    }

    ~Receiver()
    {
    }

    void run() {
        start();
        stop();
    }

private:
    void start() {
        this->mode = CLOUD;
        running = true;

        std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
        std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

        image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
        subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
        subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
        subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
        subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

        if(useExact) {
            syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
        } else {
            syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
            syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
        }
        spinner.start();

        std::chrono::milliseconds duration(1);
        while(!updateImage || !updateCloud) {
          if(!ros::ok()) {
            return;
            }
            std::this_thread::sleep_for(duration);
        }
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud->height = color.rows;
        cloud->width = color.cols;
        cloud->is_dense = false;
        cloud->points.resize(cloud->height * cloud->width);
        createLookup(this->color.cols, this->color.rows);
        
        cloudViewer();
    }

    void stop() {
        spinner.stop();

        if(useExact) {
            delete syncExact;
        } else {
            delete syncApproximate;
        }
        delete subImageColor;
        delete subImageDepth;
        delete subCameraInfoColor;
        delete subCameraInfoDepth;

        running = false;
        if(mode == BOTH) {
          imageViewerThread.join();
        }
    }

    void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
        const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {
        cv::Mat color, depth;

        readCameraInfo(cameraInfoColor, cameraMatrixColor);
        readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
        readImage(imageColor, color);
        readImage(imageDepth, depth);

        // IR image input
        if(color.type() == CV_16U) {
            cv::Mat tmp;
            color.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, color, CV_GRAY2BGR);
        }

        lock.lock();
        this->color = color;
        this->depth = depth;
        updateImage = true;
        updateCloud = true;
        lock.unlock();
    }

    void cloudViewer() {
        cv::Mat color, depth;
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
        const std::string cloudName = "rendered";
        
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        visualizer->addPointCloud(cloud, cloudName);
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(0, 0, 0);
        visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
        visualizer->setSize(color.cols, color.rows);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

        for(; running && ros::ok();) {
            if(updateCloud) {
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateCloud = false;
                lock.unlock();

                // decrare cloud_filtered 
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
                cloud_filtered->height = color.rows;
                cloud_filtered->width = color.cols;
                cloud_filtered->is_dense = false;
                cloud_filtered->points.resize(cloud_filtered->height * cloud_filtered->width);

                // create cloud from kinect
                createCloud(depth, color, cloud_filtered);

                double x1 = 0.5, y1 = 1.0, x2 = -0.5, y2 = -1.0, z1 = 2.0 , z2 = 0.0;

        // filtered (default size)
                pcl::PassThrough<pcl::PointXYZRGBA> pass;
                pass.setInputCloud(cloud_filtered);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits(x2, x1);
                pass.filter (*cloud_filtered);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits(y2, y1);
                pass.filter(*cloud_filtered);
                pass.setFilterFieldName ("z"); 
                pass.setFilterLimits(z2, z1);
                pass.filter(*cloud_filtered);

                // get space of cloth
                for (size_t i = 0; i < cloud_filtered->points.size(); i++) {
                    int r = (int) cloud_filtered->points[i].r;
                    int g = (int) cloud_filtered->points[i].g;
                    int b = (int) cloud_filtered->points[i].b;
                    if(g > 127 && g > (r+50) && g > (b+50)) {
                        if (x1 > cloud_filtered->points[i].x) x1 = cloud_filtered->points[i].x;
                        if (y1 > cloud_filtered->points[i].y) y1 = cloud_filtered->points[i].y;
                        if (z1 > cloud_filtered->points[i].z) z1 = cloud_filtered->points[i].z;
                        if (x2 < cloud_filtered->points[i].x) x2 = cloud_filtered->points[i].x;
                        if (y2 < cloud_filtered->points[i].y) y2 = cloud_filtered->points[i].y;
                        if (z2 < cloud_filtered->points[i].z) z2 = cloud_filtered->points[i].z;
                    }
                }
                // std::cout << " x1:" << x1 << " x2:" << x2 << " y1:" << y1 << " y2:" << y2 << std::endl;

                // filtered (space of cloth)
                pass.setInputCloud (cloud_filtered);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(x1, x2);
                pass.filter(*cloud_filtered);
                pass.setInputCloud(cloud_filtered);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(y1, y2);
                pass.filter(*cloud_filtered);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(z1, z2);
                pass.filter(*cloud_filtered);

                visualizer->updatePointCloud(cloud_filtered, cloudName);
            }
            visualizer->spinOnce(10);
        }
        visualizer->close();
    }

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *) {
        if(event.keyUp()) {
            switch(event.getKeyCode()) {
            case 27:
            case 'q':
                running = false; break;
            case ' ':
            case 's':
                save = true; break;
            }
        }
    }

    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const {
        cv_bridge::CvImageConstPtr pCvImage;
        pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
        pCvImage->image.copyTo(image);
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for(size_t i = 0; i < 9; ++i, ++itC) {
          *itC = cameraInfo->K[i];
        }
    }

    void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue) {
        cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
        const uint32_t maxInt = 255;

        #pragma omp parallel for
        for(int r = 0; r < in.rows; ++r) {
            const uint16_t *itI = in.ptr<uint16_t>(r);
            uint8_t *itO = tmp.ptr<uint8_t>(r);

            for(int c = 0; c < in.cols; ++c, ++itI, ++itO) {
                *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
            }
        }
        cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
    }

    void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out) {
        out = cv::Mat(inC.rows, inC.cols, CV_8UC3);
    #pragma omp parallel for
        for(int r = 0; r < inC.rows; ++r) {
            const cv::Vec3b
            *itC = inC.ptr<cv::Vec3b>(r),
            *itD = inD.ptr<cv::Vec3b>(r);
            cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

            for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO) {
                itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
                itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
                itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
            }
        }
    }

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

        #pragma omp parallel for
        for(int r = 0; r < depth.rows; ++r) {
            pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
            const uint16_t *itD = depth.ptr<uint16_t>(r);
            const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
            const float y = lookupY.at<float>(0, r);
            const float *itX = lookupX.ptr<float>();

            for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX) {
                register const float depthValue = *itD / 1000.0f;
                // Check for invalid measurements
                if(*itD == 0) {
                    // not valid
                    itP->x = itP->y = itP->z = badPoint;
                    itP->rgba = 0;
                    continue;
                }
                itP->z = depthValue;
                itP->x = *itX * depthValue;
                itP->y = y * depthValue;
                itP->b = itC->val[0];
                itP->g = itC->val[1];
                itP->r = itC->val[2];
            }
        }
    }

    void createLookup(size_t width, size_t height) {
        const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
        const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
        const float cx = cameraMatrixColor.at<double>(0, 2);
        const float cy = cameraMatrixColor.at<double>(1, 2);
        float *it;
        lookupY = cv::Mat(1, height, CV_32F);
        it = lookupY.ptr<float>();
        for(size_t r = 0; r < height; ++r, ++it) {
            *it = (r - cy) * fy;
        }
        lookupX = cv::Mat(1, width, CV_32F);
        it = lookupX.ptr<float>();
        for(size_t c = 0; c < width; ++c, ++it) {
      *it = (c - cx) * fx;
  }
}
};

int main(int argc, char **argv) {
#if EXTENDED_OUTPUT
    ROSCONSOLE_AUTOINIT;
    if(!getenv("ROSCONSOLE_FORMAT")) {
        ros::console::g_formatter.tokens_.clear();
        ros::console::g_formatter.init("[${severity}] ${message}");
    }
#endif
    ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

    if(!ros::ok()) {
        return 0;
    }

    std::string ns = K2_DEFAULT_NS;
    std::string topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
    std::string topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    bool useExact = true;
    bool useCompressed = false;

    topicColor = "/" + ns + topicColor;
    topicDepth = "/" + ns + topicDepth;
    OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
    OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

    Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

    OUT_INFO("starting receiver...");
    receiver.run();

    ros::shutdown();
    return 0;
}
