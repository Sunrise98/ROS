#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;

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

    ros::Publisher pub;

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
        pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>> ("color_filter", 1);
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
        
        cloudFilter();
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

    void cloudFilter() {
        cv::Mat color, depth;

        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        ros::Rate loop_rate(10);
        while (ros::ok()) {
            if(updateCloud) {
                lock.lock();
                color = this->color;
                depth = this->depth;
                updateCloud = false;
                lock.unlock();

                // decrare cloud
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
                cloud->height = color.rows;
                cloud->width = color.cols;
                cloud->is_dense = false;
                cloud->points.resize(cloud->height * cloud->width);

                // create cloud from kinect
                createCloud(depth, color, cloud);

                double x1 = 0.5, y1 = 1.0, x2 = -0.5, y2 = -1.0, z1 = 2.0 , z2 = 0.0;

                // filtered (default size)
                pcl::PassThrough<pcl::PointXYZRGBA> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits(x2, x1);
                pass.filter (*cloud);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits(y2, y1);
                pass.filter(*cloud);
                pass.setFilterFieldName ("z"); 
                pass.setFilterLimits(z2, z1);
                pass.filter(*cloud);

                // get space of cloth
                for (size_t i = 0; i < cloud->points.size(); i++) {
                    int r = (int) cloud->points[i].r;
                    int g = (int) cloud->points[i].g;
                    int b = (int) cloud->points[i].b;
                    if(g > 127 && g > (r+50) && g > (b+50)) {
                        if (x1 > cloud->points[i].x) x1 = cloud->points[i].x;
                        if (y1 > cloud->points[i].y) y1 = cloud->points[i].y;
                        if (z1 > cloud->points[i].z) z1 = cloud->points[i].z;
                        if (x2 < cloud->points[i].x) x2 = cloud->points[i].x;
                        if (y2 < cloud->points[i].y) y2 = cloud->points[i].y;
                        if (z2 < cloud->points[i].z) z2 = cloud->points[i].z;
                    }
                }
                // filtered (space of cloth)
                pass.setInputCloud (cloud);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(x1, x2);
                pass.filter(*cloud);
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(y1, y2);
                pass.filter(*cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(z1, z2);
                pass.filter(*cloud);

                pcl::PointCloud<pcl::PointXYZRGBA> out_cloud;
                out_cloud.height = cloud->height;
                out_cloud.width  = cloud->width;
                out_cloud.is_dense = false;
                out_cloud.points.resize(cloud->height * cloud->width);
                for (size_t i = 0; i < cloud->points.size(); ++i){
                    out_cloud.points[i] = cloud->points[i];
                }
                ROS_INFO("cloud size :%d", (int)out_cloud.points.size());
                ROS_INFO("x1: %.2lf, \tx2: %.2lf, \t size: %.2lf", x1, x2, fabs(x1-x2));
                ROS_INFO("y1: %.2lf, \ty2: %.2lf, \t size: %.2lf", y1, y2, fabs(y1-y2));
                ROS_INFO("z1: %.2lf, \tz2: %.2lf, \t size: %.2lf", z1, z2, fabs(z1-z2));

                // publish 
                sensor_msgs::PointCloud2 output;
                if (out_cloud.points.size() > 0) {
                    pcl::toROSMsg(out_cloud, output);
                    output.header.frame_id = "kinect2_rgb_optical_frame";
                    output.header.stamp = ros::Time::now();
                    pub.publish(output);
                }
            }
            loop_rate.sleep ();
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

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

        #pragma omp parallel for
        for (int r = 0; r < depth.rows; ++r) {
            pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
            const uint16_t *itD = depth.ptr<uint16_t>(r);
            const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
            const float y = lookupY.at<float>(0, r);
            const float *itX = lookupX.ptr<float>();

            for (size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX) {
                register const float depthValue = *itD / 1000.0f;
                // Check for invalid measurements
                if (*itD == 0) {
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
                itP->a = 255;
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
        for (size_t r = 0; r < height; ++r, ++it) {
            *it = (r - cy) * fy;
        }
        lookupX = cv::Mat(1, width, CV_32F);
        it = lookupX.ptr<float>();
        for (size_t c = 0; c < width; ++c, ++it) {
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
    ros::init(argc, argv, "cloth_cloud");

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