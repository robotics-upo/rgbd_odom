/**
 * @file rgbdodom.hpp
 * @brief Visual odometry based on an RGB-D sensor
 * @author Fernando Caballero, fcaballero@us.es
 * @author Francisco J Perez-Grau, fjperez@catec.aero
 * @date October 2016
 */

#ifndef __RGBDODOM_HPP__
#define __RGBDODOM_HPP__

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc.hpp> //OpenCV3
#include <opencv2/highgui.hpp> //OpenCV3
#include <opencv2/features2d.hpp> //OpenCV3
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include <algorithm>
#include "robustmatcher.hpp"
#include <cvsba/cvsba.h>


/**
 * @brief Synchronization policy for receiving RGB and depth images
 */
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;

/**
 * @brief Auxiliar function for printing tf::Transform information
 * @param transform Transform to be printed
 */
void printTF(const tf::Transform& transform)
{
    double yaw, pitch, roll;
    transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 v = transform.getOrigin();
    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
              << q.getZ() << ", " << q.getW() << "]" << std::endl
              << "            in RPY [" <<  roll*360/M_PI << ", " << pitch*360/M_PI << ", " << yaw*360/M_PI << "]" << std::endl;
}

/**
 * @brief Keypoint comparison auxiliar function to sort the sets of keypoints
 * according to their score
 * @param p1 The first keypoint
 * @param p2 The second keypoint
 * @return Boolean true if p1 > p2
 */
bool score_comparator(const cv::KeyPoint& p1, const cv::KeyPoint& p2)
{
    return p1.response > p2.response;
}

/**
 * @brief Match comparison auxiliar function to sort the sets of matches
 * @param m1 The first match
 * @param m2 The second match
 * @return Boolean true if m1 < m2
 */
bool match_comparator(const cv::DMatch& m1, const cv::DMatch& m2)
{
    return m1.distance < m2.distance;
}

/**
 * @brief KeyFrame struct for storing image, keypoints, descriptors and 3D point cloud
 */
struct KeyFrame
{
    cv::Mat img;                        // Image
    std::vector<cv::KeyPoint> kpts;     // Detected key-points
    std::vector<float> depths;          // Key-points depths
    cv::Mat desc;                       // Key-points descriptors
    std::vector<cv::Point3f> pc;        // 3D point cloud (baseFrameId_)
    std::vector<cv::Point3f> pcOdom;    // 3D point cloud (odomFrameId_)
};

/**
 * @brief SbaData struct for storing data for bundle adjustment
 */
struct SbaData
{
    std::vector<cv::Point3d> worldPoints;               // Vector of estimated 3D points (size N)
    std::vector<std::vector<cv::Point2d> > imgPoints;   // Vector of vectors of estimated image projections of 3D points (size MxN, M is number of kfs)
    std::vector<std::vector<int> > visibility;          // Element sbaVisibility[i][j] is 1 if sbaWorldPoints_[j] is visible on camera i (size MxN)
    std::vector<cv::Mat> R;                             // Vector of estimated camera rotations in Rodrigues format (size 3) (size M)
    std::vector<cv::Mat> T;                             // Vector of estimated camera traslations (size M)
    std::vector<int> buckets;
};

/**
 * @class RgbdOdom
 * @brief Estimates 6DOF pose from RGB-D data
 */
class RgbdOdom
{
public:
    /**
     * @brief Constructor
     * @param nodeName Node name for publishing topics
     * @param cameraTopic Name of the camera
     */
    RgbdOdom(std::string &nodeName, std::string &cameraTopic):
        it_(nh_),
        imageSub_(it_, cameraTopic + "/rgb/image_raw", 1),
        depthSub_(it_, cameraTopic + "/depth_registered/image_raw", 1), 
        imageSync_(syncPolicy(10), imageSub_, depthSub_),
        //fExtractor_(), //Changed for OpenCV3
        //fDetector_(), //Changed for OpenCV3
        odomInit_(false),
        calibInit_(false),
        imuReceived_(false),
        rxImu_(0.),
        ryImu_(0.)
	{
		// Topic subscription
        cInfoSub_ = nh_.subscribe(cameraTopic + "/rgb/camera_info", 1, &RgbdOdom::cInfoCallback, this);
        imageSync_.registerCallback(boost::bind(&RgbdOdom::syncImageCallback, this, _1, _2));

		// Load node parameters
        ros::NodeHandle lnh("~");
        std::string tfTopic;
        if(!lnh.getParam("tf_topic_out", tfTopic))
            tfTopic = "/rgbd_transform";
        if(!lnh.getParam("max_features", maxFeatures_))
            maxFeatures_ = 200;
        if(maxFeatures_ < 200)
        {
            maxFeatures_ = 200;
            ROS_WARN("max_features should be above 200. Setting default value (200)");
        }
        if(!lnh.getParam("flow_threshold", flowThreshold_))
            flowThreshold_ = 15;
        if(!lnh.getParam("min_matches", minMatches_))
            minMatches_ = 10;
        if(!lnh.getParam("odom_frame_id", odomFrameId_))
            odomFrameId_ = "odom";
        if(!lnh.getParam("base_frame_id", baseFrameId_))
            baseFrameId_ = "base_link";
        if(!lnh.getParam("publish_point_cloud", pubPoints_))
            pubPoints_ = false;
        if(!lnh.getParam("publish_image", publishImage_))
            publishImage_ = false;
        if(!lnh.getParam("use_sba", useSba_))
            useSba_ = false;
        if(!lnh.getParam("type_sba", typeSba_))
            typeSba_ = 2;
        if(!lnh.getParam("max_keyframes", maxKeyframes_))
            maxKeyframes_ = 4;
        if(!lnh.getParam("use_imu", useImu_))
            useImu_ = false;
        std::string imuTopic;
        if(!lnh.getParam("imu_topic", imuTopic))
            imuTopic = "/imu";

        ROS_INFO_STREAM("[RgbdOdom] Subscribed to:\n\t" << imageSub_.getTopic() << "\n\t"
                        << depthSub_.getTopic() << "\n\t" << cInfoSub_.getTopic());
        if(useImu_)
        {
            imuSub_ = nh_.subscribe(imuTopic, 1, &RgbdOdom::imuCallback, this);
            ROS_INFO_STREAM("[RgbdOdom] Subscribed to: " << imuTopic);
        }

        // Initialize odometry
        odom_.setIdentity();
        odomK_.setIdentity();
        odomBase_.setIdentity();
		tfCache_ = false;

        // Advertise images
        if(publishImage_)
            imgPub_ = it_.advertise(nodeName+"/image", 1);
            
        // Advertise the new point-cloud
        if(pubPoints_)
            pcPub_ = nh_.advertise<sensor_msgs::PointCloud2>(nodeName+"/point_cloud", 1);

        // Advertise transform topic
        tfPub_ = nh_.advertise<geometry_msgs::TransformStamped>(tfTopic, 1);
    }

    /** @brief Destructor */
    ~RgbdOdom(void)
    {
    }

private:
	
    void imuCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,msg->orientation.w);
        tf::Matrix3x3 m(q);
        double rzImu;
        m.getRPY(rxImu_, ryImu_, rzImu);
        //std::cout << "IMU: " << rxImu_ << ", " << ryImu_ << ", " << rzImu << std::endl;

        if(!imuReceived_)
            imuReceived_ = true;
    }

    /**
     * @brief Convert images from ROS message to OpenCV format (cv::Mat)
     * @param rgbMsg RGB image in ROS format
     * @param depthMsg Depth image in ROS format
     * @param rgbMat RGB image in OpenCV format
     * @param depthMat Depth image in OpenCV format
     * @return
     */
    bool convertImages(const sensor_msgs::ImageConstPtr& rgbMsg,
                       const sensor_msgs::ImageConstPtr& depthMsg,
                       cv::Mat& rgbMat, cv::Mat& depthMat)
    {
        // Convert to OpenCV format without copy
        cv_bridge::CvImageConstPtr cvbRgb, cvbDepth;
        try
        {
            cvbRgb = cv_bridge::toCvCopy(rgbMsg);
            cvbDepth = cv_bridge::toCvCopy(depthMsg);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }

        // Store header and encoding
        imgHeader_ = cvbRgb->header;
        imgEncodings_ = cvbRgb->encoding;

        cvbRgb->image.copyTo(rgbMat);
        cvbDepth->image.copyTo(depthMat);

        // Store image size
        imgSize_.width = rgbMat.cols;
        imgSize_.height = rgbMat.rows;

        return true;
    }

    /**
     * @brief Distribute max_features among all buckets (six buckets, 2rows x 3cols)
     * @param srcKpts Sorted key-points
     * @param dstKpts Output bucketed key-points
     */
    void kptsBucketing(std::vector<cv::KeyPoint>& srcKpts, std::vector<cv::KeyPoint>& dstKpts)
    {
        const int maxFeatBuck = maxFeatures_/6;
        int buckets[6] = {maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck};
        dstKpts.clear();
        for(int i=0; i<srcKpts.size(); i++)
        {
            int id;
            if(srcKpts[i].pt.y <= imgSize_.height/2)
                id = 0;
            else
                id = 3;
            if(srcKpts[i].pt.x <= imgSize_.width/3)
                id += 0;
            else if(srcKpts[i].pt.x <= 2*imgSize_.width/3)
                id += 1;
            else
                id += 2;

            if(buckets[id] > 0)
            {
                buckets[id]--;
                dstKpts.push_back(srcKpts[i]);
            }
        }
    }

    /**
     * @brief Detects key-points in the RGB image, applying bucketing
     * @param img Input image
     * @param kpts Key-points extracted from input image
     */
    void selectKeypoints(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts)
    {
        // Detect key-points in the image
        std::vector<cv::KeyPoint> kpts_all;
		
        //fDetector_.detect(img, kpts_all); //OpenCV2
		fDetector_->detect(img, kpts_all);  //OpenCV3

        // Sort keypoints according to their score
        std::sort(kpts_all.begin(), kpts_all.end(), score_comparator);

        // Distribute maxFeatures_ among buckets
        kptsBucketing(kpts_all, kpts);
    }

    /** @brief Compute feature flow between matched key-points
     * @param matches List of matches
     * @param trainKpts Key-points from 'train' image
     * @param queryKpts Key-points fraom 'query' image
     * @return Computed feature flow
     */
    double computeFeatureFlow(const std::vector<cv::DMatch> &matches,
                              const std::vector<cv::KeyPoint> &trainKpts,
                              const std::vector<cv::KeyPoint> &queryKpts)
    {
        int idT, idQ;
        double xDiff, yDiff, totalFlow = 0.0;

        for(size_t i=0; i<matches.size(); i++)
        {
            idT = matches[i].trainIdx;
            idQ = matches[i].queryIdx;
            xDiff = trainKpts[idT].pt.x - queryKpts[idQ].pt.x;
            yDiff = trainKpts[idT].pt.y - queryKpts[idQ].pt.y;
            totalFlow += sqrt(xDiff * xDiff + yDiff * yDiff);
        }

        return totalFlow / matches.size();
    }

    /**
     * @brief Match key-points and find correspondences between previous 3D points and current projections
     * @param keypoints[in] Key-points detected in current image
     * @param descriptors[in] Descriptors or detected key-points
     * @param kFrame[in] Structure holding last KeyFrame data
     * @param flow[out] Feature flow between last keyframe and current image
     * @param points3d[out] 3D points from last keyframe matched to current image
     * @param projections[out] 2D projections of matched 3D points
     */
    bool frameMatching(std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                       KeyFrame& kFrame, double& flow, std::vector<cv::Point3f>& points3d,
                       std::vector<cv::Point2f>& projections, std::vector<cv::DMatch>& matchesOut)
    {
        // Frame to frame matching
        std::vector<cv::DMatch> matches;
        try
        {
            matcher_.match(keypoints, descriptors, kFrame.kpts, kFrame.desc, matches);
        }
        catch(std::exception e)
        {
            ROS_ERROR("Matching error!");
            odomInit_ = false;
            return false;
        }

        // Compute features flow
        flow = computeFeatureFlow(matches, kFrame.kpts, keypoints);

        // Establish matching between current and previous image pairs
        for(size_t i=0; i<matches.size(); i++)
        {
            const cv::DMatch& match = matches[i];
            if(kFrame.depths[match.trainIdx] > 0.3 && kFrame.depths[match.trainIdx] < 8.0)
            {
                points3d.push_back(kFrame.pc[match.trainIdx]);
                projections.push_back(keypoints[match.queryIdx].pt);
                matchesOut.push_back(match);
            }
        }

        return true;
    }

    /** @brief Converts OpenCV translation and rotation into tf::Transform
     * @param[in] t OpenCV translation vector
     * @param[in] R OpenCV rotation matrix
     * @param[out] transform ROS tf::Transform element
     */
    void openCVToTf(const cv::Mat& t, const cv::Mat& R, tf::Transform& tf)
    {
        tf::Vector3 translation_tf(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0));

        tf::Matrix3x3 rotation_tf;
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                rotation_tf[i][j] = R.at<double>(i,j);

        tf.setOrigin(translation_tf);
        tf.setBasis(rotation_tf);
    }

    /** @brief Converts OpenCV translation and rotation into tf::Transform
     * @param[in] t OpenCV translation vector
     * @param[in] R OpenCV rotation matrix
     * @param[out] transform ROS tf::Transform element
     */
    void tfToOpenCV(const tf::Transform& tf, cv::Mat& t, cv::Mat& R)
    {
        t.at<double>(0,0) = tf.getOrigin().getX();
        t.at<double>(1,0) = tf.getOrigin().getY();
        t.at<double>(2,0) = tf.getOrigin().getZ();

        tf::Matrix3x3 rotationTf = tf.getBasis();
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                R.at<double>(i,j) = rotationTf[i][j];
    }

    /** @brief Solve PnP problem between last keyframe and current image
     * @param[out] points3d 3D points matched between previous and current left images, taken from previous 3D point cloud
     * @param[out] projections 2D projections of matched 3D points in current image
     * @param[out] deltaT Estimated transform between previous and current frames
     */
    bool leftPnP(std::vector<cv::Point3f>& points3d, std::vector<cv::Point2f>& projections,
                 tf::Transform& deltaT)
    {
        cv::Mat R, r, T, inliers;

        // Solve PnP using RANSAC and EPNP algorithm
        try
        {
            //cv::solvePnPRansac(points3d, projections, K_, cv::Mat(), r, T, false, 150, 2, 100, inliers, CV_EPNP); //OpenCV2
			cv::solvePnPRansac(points3d, projections, K_, cv::Mat(), r, T, false, 150, 2, 0.99, inliers, CV_EPNP);  //OpenCV3
			//cv::solvePnPRansac(board, points[index], cameraMatrix, distortion, rvec, translation, false, 300, 0.05, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE); //OpenCV3
            cv::Rodrigues(r, R);
        }
        catch(std::exception e)
        {
            ROS_ERROR("PnP error!");
            odomInit_ = false;
            return false;
        }

        // Convert to tf::Transform
        openCVToTf(T, R, deltaT);

        return true;
    }

    /**
     * @brief Published image showing keypoints and feature matching results
     * @param img Input image
     * @param kpts Key-points from input image
     * @param matches Matches between current image key-points and last key-frame
     */
    void publishImage(const cv::Mat &img, const std::vector<cv::KeyPoint> &kpts, const std::vector<cv::DMatch>& matches)
    {
        for(size_t i=0; i<kpts.size(); i++)
            cv::circle(image_, kpts[i].pt, 1, cv::Scalar(255,0,0), 2);
//        for(size_t i=0; i<matches.size(); i++)
//            cv::circle(image_, kpts[matches[i].queryIdx].pt, 1, cv::Scalar(0,255,0), 2);
        cv_bridge::CvImage send_img(imgHeader_, imgEncodings_, image_);
        imgPub_.publish(send_img.toImageMsg());
    }
    
    /**
     * @brief Publishes full sensor point cloud in the RGB camera frame
     * @param img Input depth image
     * @param frame_id Message frame name
     * @param stamp Time stamp to associate to the point cloud message
     */
    void publishFullPointCloud(const cv::Mat img, std::string frame_id, ros::Time stamp)
    {
        sensor_msgs::PointCloud outCloud;
        sensor_msgs::PointCloud2 outCloud2;

        geometry_msgs::Point32 pt;
        outCloud.header.frame_id = frame_id;
        outCloud.header.stamp = stamp;
        outCloud.points.clear();
        float kx = 1.0/K_.at<double>(0,0), cx = K_.at<double>(0,2), ky = 1.0/K_.at<double>(1,1), cy = K_.at<double>(1,2);
        for(int v=0; v<img.rows; v++)
        {
            for(int u=0; u<img.cols; u++)
            {
                pt.z = img.at<float>(v, u);
                if(pt.z > 0.3 && pt.z < 8.0)
                {
                    pt.x = (float)(u-cx) * pt.z * kx;
                    pt.y = (float)(v-cy) * pt.z * ky;
                    outCloud.points.push_back(pt);
                }
            }
        }

        sensor_msgs::convertPointCloudToPointCloud2(outCloud, outCloud2);
		pcPub_.publish(outCloud2);
    }

    /**
     * @brief Translates and rotates a point cloud according to the provided transform
     * @param transform Transformation to apply
     * @param cloudIn Input point cloud
     * @param cloudOut Output point cloud
     */
    void transformCloud(const tf::Transform& transform, const std::vector<cv::Point3f>& cloudIn,
                        std::vector<cv::Point3f>& cloudOut)
    {
        tf::Vector3 origin = transform.getOrigin();
        tf::Matrix3x3 basis = transform.getBasis();

        unsigned int length = cloudIn.size();
        cloudOut.resize(length);

        // Transform points
        for (unsigned int i = 0; i < length ; i++)
        {
            if(!cvIsNaN(cloudIn[i].x))
            {
                double x = basis[0].x() * cloudIn[i].x + basis[0].y() * cloudIn[i].y + basis[0].z() * cloudIn[i].z + origin.x();
                double y = basis[1].x() * cloudIn[i].x + basis[1].y() * cloudIn[i].y + basis[1].z() * cloudIn[i].z + origin.y();
                double z = basis[2].x() * cloudIn[i].x + basis[2].y() * cloudIn[i].y + basis[2].z() * cloudIn[i].z + origin.z();
                cloudOut[i] = cv::Point3f(x,y,z);
            }
            else
            {
                cloudOut[i] = cloudIn[i];
            }
        }
    }

    /**
     * @brief Updates key-frame data in kFrame_ class member
     * @param rgbImg RGB image
     * @param depthImg Depth image
     * @param kpts Key-points detected in RGB image
     * @param desc Key-points descriptors
     */
    void updateKeyframe(const cv::Mat& rgbImg, const cv::Mat& depthImg, const std::vector<cv::KeyPoint>& kpts, const cv::Mat& desc)
    {
        // Save current images as previous ones
        rgbImg.copyTo(kFrame_.img);

        // Save current kpts and descriptors as previous
        kFrame_.kpts = kpts;
        desc.copyTo(kFrame_.desc);

        // Compute the associated point-cloud
        kFrame_.pc.clear();
        kFrame_.depths.clear();
        float kx = 1.0/K_.at<double>(0,0), cx = K_.at<double>(0,2), ky = 1.0/K_.at<double>(1,1), cy = K_.at<double>(1,2);
        for(size_t i=0; i<kpts.size(); i++)
        {
            float d;
            cv::Point3f p;
            const cv::Point2f& pt = kpts[i].pt;
            d = depthImg.at<float>((int)pt.y, (int)pt.x);

            p.z = d;
            p.x = (float)(pt.x-cx) * d * kx;
            p.y = (float)(pt.y-cy) * d * ky;
            kFrame_.pc.push_back(p);                // it may be NaN but we store it so they match the keypoints vector
            kFrame_.depths.push_back(d);
        }

        // Transform cloud to odom frame
        kFrame_.pcOdom.clear();
        transformCloud(odomK_, kFrame_.pc, kFrame_.pcOdom);
    }

    /**
     * @brief updateSbaData
     * @param kFrame
     * @param points3d
     * @param points2d
     */
    void updateSbaData(const tf::Transform& odom, const KeyFrame& kFrame,
                       std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& keypoints)
    {
        // Add key-frame rotation and translation
        if(sbaData_.R.empty())
        {
            //cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
            //sbaData_.R.push_back(R);
            sbaData_.R.push_back(cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0)));
            sbaData_.T.push_back(cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0)));
        }
        cv::Mat rot(3, 3, CV_64FC1, cv::Scalar::all(0));
        cv::Mat T(3, 1, CV_64FC1, cv::Scalar::all(0));
        tfToOpenCV(odom.inverse(), T, rot);             // invert our odom so it fits OpenCV
        cv::Mat R(3, 1, CV_64FC1, cv::Scalar::all(0));
        cv::Rodrigues(rot, R);
        sbaData_.R.push_back(R);
        //sbaData_.R.push_back(rot);
        sbaData_.T.push_back(T);
        //std::cout << "R: " << rot << std::endl;
        //std::cout << "T: " << T << std::endl;

        // Sort matches
        std::sort(matches.begin(), matches.end(), match_comparator);

        // First data filling with 2 cameras
        if(matchData_.worldPoints.empty())
        {
            matchData_.worldPoints.resize(matches.size());
            matchData_.imgPoints.resize(2);
            matchData_.visibility.resize(2);
            for(size_t i=0; i<matches.size(); i++)
            {
                matchData_.imgPoints[0].resize(matches.size());
                matchData_.imgPoints[1].resize(matches.size());
                matchData_.visibility[0].resize(matches.size());
                matchData_.visibility[1].resize(matches.size());

                const cv::DMatch& match = matches[i];

                // 3D point in odom frame and projections on images
                cv::Point3d wp = kFrame.pcOdom[match.trainIdx];
                cv::Point2d pxPrev = kFrame.kpts[match.trainIdx].pt;
                cv::Point2d pxCurr = keypoints[match.queryIdx].pt;

                matchData_.worldPoints[i] = wp;
                matchData_.imgPoints[0][i] = pxPrev;
                matchData_.imgPoints[1][i] = pxCurr;
                matchData_.visibility[0][i] = 1;
                matchData_.visibility[1][i] = 1;
            }
            //std::cout << "First keyframe match: " << matchData_.worldPoints.size() << " points" << std::endl;
        }
        else
        {
            // Check for same keypoints between previous key-frame and last stored imgPoints
            std::vector<cv::Point2d> newImgPoints(matchData_.worldPoints.size(), cv::Point2d(0,0));
            std::vector<int> newVisibility(matchData_.worldPoints.size(), 0);

            // Take last key-frame's imgPoints vector to check with pxPrev
            std::vector<cv::Point2d> lastImgPoints = matchData_.imgPoints[matchData_.imgPoints.size()-1];

            // Calculate ideal projections of 3D points in current image
            std::vector<cv::Point2d> imgProj;
            cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64FC1);
            cameraMatrix.ptr<double>(0)[0] = cameraMatrix.ptr<double>(0)[4] = 570.3422241210938;
            cameraMatrix.ptr<double>(0)[2] = 319.5;
            cameraMatrix.ptr<double>(0)[5] = 239.5;
            cv::Mat distCoeffs = cv::Mat(5,1,CV_64FC1, cv::Scalar::all(0));
            cv::projectPoints(matchData_.worldPoints, sbaData_.R.back(), sbaData_.T.back(), cameraMatrix, distCoeffs, imgProj);

            for(size_t i=0; i<matches.size(); i++)
            {
                const cv::DMatch& match = matches[i];
                cv::Point3d wp = kFrame.pcOdom[match.trainIdx];
                //std::cout << "Keyframe points\n\t" << kFrame.pcOdom[match.trainIdx];
                cv::Point2d pxPrev = kFrame.kpts[match.trainIdx].pt;
                cv::Point2d pxCurr = keypoints[match.queryIdx].pt;

                int pos = 0;
                for(; pos<lastImgPoints.size(); pos++)
                {
                    // If same pixel, add correspondence and turn visibility ON
                    if(pxPrev == lastImgPoints[pos])
                    {
                        // If same 3D point (+/- 10cm)
                        if(cv::norm(wp - matchData_.worldPoints[pos]) < 0.1)
                        {
                            // If projection fits
                            if(cv::norm(pxCurr - imgProj[pos]) < 10)
                            {
                                //std::cout << "\t\tFound match with " << matchData_.worldPoints[pos] << std::endl;
                                newImgPoints[pos] = pxCurr;
                                newVisibility[pos] = 1;
                                break;
                            }
                        }
                    }
                }
                if(pos == lastImgPoints.size())   // If not found, add it to all relevant vectors
                {
                    newImgPoints.push_back(pxCurr);
                    newVisibility.push_back(1);
                    matchData_.worldPoints.push_back(wp);
                    for(size_t k=0; k<matchData_.imgPoints.size(); k++)
                    {
                        matchData_.imgPoints[k].push_back(cv::Point2d(0,0));
                        matchData_.visibility[k].push_back(0);
                    }
                }
            }
            matchData_.imgPoints.push_back(newImgPoints);
            matchData_.visibility.push_back(newVisibility);


            // Remove first key-frame if maxKeyframes_ reached
            if(sbaData_.R.size() > maxKeyframes_)
            {
                sbaData_.R.erase(sbaData_.R.begin());
                sbaData_.T.erase(sbaData_.T.begin());

                std::vector<cv::Point3d> newSbaWorldPoints;
                std::vector<std::vector<cv::Point2d> > newSbaImgPoints;
                std::vector<std::vector<int> > newSbaVisibility;
                newSbaImgPoints.resize(maxKeyframes_);
                newSbaVisibility.resize(maxKeyframes_);

                //std::cout << "Current wps: " << matchData_.worldPoints.size() << std::endl;

                // Check if visible worldPoints from first key-frame are present in the rest of key-frames
                // If they are not, do not keep it in worldPoints
                //std::cout << "Check wps from first kf\n";
                std::vector<cv::Point2d> firstImgPoints = matchData_.imgPoints[0];
                std::vector<int> firstVisibility = matchData_.visibility[0];
                for(size_t i=0; i<firstImgPoints.size(); i++)
                {
                    if(firstVisibility[i] == 1)
                    {
                        bool isSomewhere = false;
                        for(size_t j=1; j<matchData_.visibility.size(); j++)
                        {
                            if(matchData_.visibility[j][i] == 1)
                            {
                                isSomewhere = true;
                                break;
                            }
                        }

                        if(isSomewhere) // if found, add it
                        {
                            //Add it to newWorldPoints
                            newSbaWorldPoints.push_back(matchData_.worldPoints[i]);
                            for(size_t k=1; k<matchData_.imgPoints.size(); k++)
                            {
                                newSbaImgPoints[k-1].push_back(matchData_.imgPoints[k][i]);
                                newSbaVisibility[k-1].push_back(matchData_.visibility[k][i]);
                            }
                        }
                    }
                    else // if not visible in first key-frame, add it because it's visible from other key-frame
                    {
                        newSbaWorldPoints.push_back(matchData_.worldPoints[i]);
                        for(size_t k=1; k<matchData_.imgPoints.size(); k++)
                        {
                            newSbaImgPoints[k-1].push_back(matchData_.imgPoints[k][i]);
                            newSbaVisibility[k-1].push_back(matchData_.visibility[k][i]);
                        }
                    }
                }

                matchData_.worldPoints.clear();
                matchData_.imgPoints.clear();
                matchData_.visibility.clear();
                matchData_.worldPoints = newSbaWorldPoints;
                matchData_.imgPoints = newSbaImgPoints;
                matchData_.visibility = newSbaVisibility;

                //std::cout << "Final wps: " << matchData_.worldPoints.size() << std::endl;
            }

            // Take only points visible in all images
            std::vector<int> valid_points;
            int num_points = matchData_.imgPoints[0].size();
            int num_images = matchData_.imgPoints.size();
            for(size_t i=0; i<num_points; i++)
            {
                int visib = 0;
                for(size_t j=0; j<num_images; j++)
                {
                    if(matchData_.visibility[j][i])
                        visib++;
                }
                if(visib >= (int)(maxKeyframes_/4))
                    valid_points.push_back(i);
            }

            // Bucket the surviving points (7x4), and add points if needed
            sbaData_.buckets.clear();
            sbaData_.worldPoints.clear();
            sbaData_.imgPoints.clear();
            sbaData_.visibility.clear();
            sbaData_.buckets.resize(28, 5);
            sbaData_.imgPoints.resize(num_images);
            sbaData_.visibility.resize(num_images);
            for(int i=0; i<valid_points.size(); i++)
            {
                cv::Point2d pt = matchData_.imgPoints[num_images-1][valid_points[i]];
                int id;
                if(pt.y <= imgSize_.height/4)
                    id = 0;
                else if(pt.y <= imgSize_.height/2)
                    id = 7;
                else if(pt.y <= 3*imgSize_.height/4)
                    id = 14;
                else
                    id = 21;

                if(pt.x < imgSize_.width/7)
                    id += 0;
                else if(pt.x <= 2*imgSize_.width/7)
                    id += 1;
                else if(pt.x <= 3*imgSize_.width/7)
                    id += 2;
                else if(pt.x <= 4*imgSize_.width/7)
                    id += 3;
                else if(pt.x <= 5*imgSize_.width/7)
                    id += 4;
                else if(pt.x <= 6*imgSize_.width/7)
                    id += 5;
                else
                    id += 6;

                if(sbaData_.buckets[id] > 0)
                {
                    sbaData_.buckets[id]--;
                    sbaData_.worldPoints.push_back(matchData_.worldPoints[valid_points[i]]);
                    for(size_t j=0; j<num_images; j++)
                    {
                        sbaData_.imgPoints[j].push_back(matchData_.imgPoints[j][valid_points[i]]);
                        sbaData_.visibility[j].push_back(matchData_.visibility[j][valid_points[i]]);
                    }
                    //if(matchData_.visibility[num_images-1][valid_points[i]] == 1)
                    //    cv::circle(image_, pt, 1, cv::Scalar(0,255,0), 3);
                }
            }
            /*std::cout << "Total points: " << matchData_.worldPoints.size() <<
                         ", valid points (" << (int)(maxKeyframes_/4) << "): " << valid_points.size() <<
                         ", bucketed points: " << sbaData_.worldPoints.size() << std::endl;*/

//            for(size_t i=0; i<10; i++)
//                std::cout << "Point " << i << ": " << sbaData_.worldPoints[i] << std::endl;
        }
    }

    /**
   * @brief runSBA
   */
    bool runSBA()
    {
        if(sbaData_.R.size() < 3 || sbaData_.worldPoints.size() < 5)
            return false;
//        for(size_t i=0; i<sbaData_.T.size(); i++)
//        {
//            std::cout << "T[" << i << "]: " << sbaData_.T[i].at<double>(0,0) << "," << sbaData_.T[i].at<double>(0,1) << "," << sbaData_.T[i].at<double>(0,2) << std::endl;
//            std::cout << "R[" << i << "]: " << sbaData_.R[i] << std::endl;//.at<double>(0,0) << "," << sbaData_.R[i].at<double>(0,1) << "," << sbaData_.R[i].at<double>(0,2) << std::endl;
//        }

        std::vector<cv::Mat> cameraMatrix, distCoeffs;
        int NCAMS = sbaData_.R.size(); 	// number of cameras

        // fill camera intrinsics (same intrinsics for all cameras)
        cameraMatrix.resize(NCAMS);
        for(int i=0; i<NCAMS; i++)
        {
            cameraMatrix[i] = cv::Mat::eye(3,3,CV_64FC1);
            cameraMatrix[i].ptr<double>(0)[0] = cameraMatrix[i].ptr<double>(0)[4] = 570.3422241210938;
            cameraMatrix[i].ptr<double>(0)[2] = 319.5;
            cameraMatrix[i].ptr<double>(0)[5] = 239.5;
        }

        // fill distortion (assume no distortion)
        distCoeffs.resize(NCAMS);
        for(int i=0; i<NCAMS; i++)
            distCoeffs[i] = cv::Mat(5,1,CV_64FC1, cv::Scalar::all(0));

//        std::vector<std::vector<cv::Point2d> > imgP;
//        imgP.resize(sbaData_.R.size());
//        for(int i=0; i<sbaData_.R.size(); i++)
//            cv::projectPoints(sbaData_.worldPoints, sbaData_.R[i], sbaData_.T[i], cameraMatrix[i], distCoeffs[i], imgP[i]);
//        for(int i=0; i<sbaData_.worldPoints.size(); i++)
//        {
//            std::cout << sbaData_.worldPoints[i] << " is projected in " << std::endl;
//            for(int j=0; j<sbaData_.R.size(); j++)
//            {
//                std::cout << "\tcam" << j << ": " << imgP[j][i] << " but SBA is given " << sbaData_.imgPoints[j][i] << "(" << sbaData_.visibility[j][i] << ")";
//                if(cv::norm(imgP[j][i] - sbaData_.imgPoints[j][i]) > 10)
//                    std::cout << " DIFF";
//                std::cout << std::endl;
//            }
//            std::cout << std::endl;
//        }

        //Clock t;
        //std::cout << "Launching SBA with " << sbaData_.worldPoints.size() << " points and " << sbaData_.R.size() << " cameras" << std::endl;
        cvsba::Sba sba;
        cvsba::Sba::Params params ;
        switch(typeSba_)
        {
        case 0:
            params.type = cvsba::Sba::MOTIONSTRUCTURE;
            break;
        case 1:
            params.type = cvsba::Sba::MOTION;
            break;
        case 2:
            params.type = cvsba::Sba::STRUCTURE;
            break;
        default:
            params.type = cvsba::Sba::STRUCTURE;
            break;
        }
        params.iterations = 150;
        params.minError = 1e-10;
        params.fixedIntrinsics = 5;
        params.fixedDistortion = 5;
        params.verbose = false;
        sba.setParams(params);
        std::vector<cv::Point3d> worldPointsPrev = sbaData_.worldPoints;
        try{
            sba.run(sbaData_.worldPoints,  sbaData_.imgPoints,  sbaData_.visibility,  cameraMatrix, sbaData_.R,  sbaData_.T, distCoeffs);
        }
        catch(const std::exception e){
            return false;
        }
        //std::cout << "Initial error=" << sba.getInitialReprjError() << ". Final error=" << sba.getFinalReprjError() << std::endl;
        //std::cout << "Run Time: " << t.tock() << std::endl;
//        for(size_t i=0; i<sbaData_.T.size(); i++)
//        {
//            std::cout << "T[" << i << "]: " << sbaData_.T[i].at<double>(0,0) << "," << sbaData_.T[i].at<double>(0,1) << "," << sbaData_.T[i].at<double>(0,2) << std::endl;
//            std::cout << "R[" << i << "]: " << sbaData_.R[i] << std::endl;//.at<double>(0,0) << "," << sbaData_.R[i].at<double>(0,1) << "," << sbaData_.R[i].at<double>(0,2) << std::endl;
//        }

//        std::vector<std::vector<cv::Point2d> > imgP2;
//        imgP2.resize(sbaData_.R.size());
//        for(int i=0; i<sbaData_.R.size(); i++)
//            cv::projectPoints(sbaData_.worldPoints, sbaData_.R[i], sbaData_.T[i], cameraMatrix[i], distCoeffs[i], imgP2[i]);
//        for(int i=0; i<sbaData_.worldPoints.size(); i++)
//        {
//            std::cout << sbaData_.worldPoints[i] << " (formerly " << worldPointsPrev[i] << ") is projected in " << std::endl;
//            for(int j=0; j<sbaData_.R.size(); j++)
//            {
//                std::cout << "\tcam" << j << ": " << imgP2[j][i] << " but SBA is given " << sbaData_.imgPoints[j][i] << "(" << sbaData_.visibility[j][i] << ")";
//                if(cv::norm(imgP2[j][i] - sbaData_.imgPoints[j][i]) > 10)
//                    std::cout << " DIFF";
//                std::cout << std::endl;
//            }
//            std::cout << std::endl;
//        }

        return true;
    }

    /**
        * @brief updateOdom
        * @param data
        * @param odomTf
        */
    void updateOdom(const SbaData& data, tf::Transform& odomTf)
    {
        // Take last rotation and translation
        cv::Mat R = data.R[data.R.size()-1];
        cv::Mat rot(3, 3, CV_64FC1, cv::Scalar::all(0));
        cv::Rodrigues(R, rot);
        cv::Mat T = data.T[data.T.size()-1];
        tf::Transform tfInv;
        openCVToTf(T, rot, tfInv);
        odomTf = tfInv.inverse();
        //std::cout << "Updated" << std::endl;
    }

    /**
     * @brief Compensate IMU roll and pitch angles in base frame
     * @param odom
     */
    void compensateIMU(tf::Transform& odom)
    {
        double rxOdom, ryOdom, rzOdom;
        tf::Matrix3x3(odom.getRotation()).getRPY(rxOdom, ryOdom, rzOdom);
        tf::Quaternion qCompensated;
        qCompensated.setRPY(rxImu_, ryImu_, rzOdom);
        odom.setRotation(qCompensated);
    }

    /**
     * @brief Synchronized RGB and Depth images callback
     * @param imgMsg RGB image message
     * @param depthMsg Depth image message
     */
    void syncImageCallback(const sensor_msgs::ImageConstPtr& imgMsg, const sensor_msgs::ImageConstPtr& depthMsg)
    {
     
        double flow = 0.0;
        //Clock t;
									
        // Processing does not start until first calibration is received
        if(!calibInit_)
        {
            ROS_WARN("No camera calibration received yet, skipping image processing");
            return;
        }

        // Convert to OpenCV format
        cv::Mat rgbImg, depthImg;
        if(!convertImages(imgMsg, depthMsg, rgbImg, depthImg))
            return;
        rgbImg.copyTo(image_);

        // Detect key-points in the image
        std::vector<cv::KeyPoint> kpts;
        selectKeypoints(rgbImg, kpts);


        // Extract feature descritors from image
		cv::Mat desc;
        //fExtractor_.compute(rgbImg, kpts, desc); //OpenCV2
		fExtractor_->compute(rgbImg, kpts, desc);  //OpenCV3

        
        // Pre-cache transform from camera to base frame (this is done just once!)
		if(!tfCache_)
		{	
			try
			{
				tfListener_.waitForTransform(baseFrameId_, imgMsg->header.frame_id, ros::Time(0), ros::Duration(2.0));
				tfListener_.lookupTransform(baseFrameId_, imgMsg->header.frame_id, ros::Time(0), Tcam2base_);
				tfCache_ = true;
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("[RgbdOdom] %s",ex.what());
				return;
			}
		}
		
        // Compute odometry
        if(odomInit_)
        {
            // Matching between previous key-frame and current image
            std::vector<cv::Point3f> points3d;
            std::vector<cv::Point2f> projections;
            std::vector<cv::DMatch> matches;
            if(!frameMatching(kpts, desc, kFrame_, flow, points3d, projections, matches))
                return;

            // If there are enough matches
            //std::cout << "Matches: " << points3d.size() << " (min: " << minMatches_ << ")" << std::endl;
            if(points3d.size() >= minMatches_)
            {
                // Compute PnP between previous and current frames
                tf::Transform deltaT;
                if(!leftPnP(points3d, projections, deltaT))
                    return;
                //std::cout << "deltaT" << std::endl;
                //printTF(deltaT);

                // Check for wrong estimations and set to zero in case of error (larger margin in Y, vertical axis)
                if(fabs(deltaT.getOrigin().getX()) < 0.4 &&
                        fabs(deltaT.getOrigin().getY()) < 0.4 &&
                        fabs(deltaT.getOrigin().getZ()) < 0.2)
                {

                    // Compute odometry
                    //std::cout << "Flow: " << flow << " (thres: " << flowThreshold_ << ")" << std::endl;
                    if(flow > flowThreshold_)
                    {
                        //std::cout << "New key-frame" << std::endl;

                        // Compute new key-frame odom
                        odomK_ = odomK_ * deltaT.inverse();

                        // Update odom
                        odom_ = odomK_;

                        if(useSba_)
                        {
                            // Update SBA data
                            updateSbaData(odomK_, kFrame_, matches, kpts);

                            if(runSBA())
                                updateOdom(sbaData_, odomK_);

                        }

                        if(publishImage_)
                            publishImage(rgbImg, kpts, matches);
                    }
                    else
                    {
                        odom_ = odomK_ * deltaT.inverse();
                    }
                    //std::cout << "odom" << std::endl;
                    //printTF(odom_);
                }
                else
                    ROS_WARN_STREAM("Odometry jump detected");
            }
            else
                ROS_WARN_STREAM("Not enough matches! (" << points3d.size() << ")");

            // Publish odom and TF
            // Transform odom_ from camera frame to base_link frame
            odomBase_ = Tcam2base_*odom_*Tcam2base_.inverse();

            // Compensate IMU roll and pitch angles
            if(useImu_ && imuReceived_)
                compensateIMU(odomBase_);

            // Transform back to compensate also odom_ and odomK_
            odom_ = Tcam2base_.inverse()*odomBase_*Tcam2base_;
            if(flow > flowThreshold_)
                odomK_ = odom_;

            // Publish Odometry in odom frame
            geometry_msgs::TransformStamped odomBaseTf;
            odomBaseTf.header.stamp = imgMsg->header.stamp;
            odomBaseTf.header.frame_id = odomFrameId_;
            odomBaseTf.child_frame_id = baseFrameId_;
            odomBaseTf.transform.translation.x = odomBase_.getOrigin().x();
            odomBaseTf.transform.translation.y = odomBase_.getOrigin().y();
            odomBaseTf.transform.translation.z = odomBase_.getOrigin().z();
            odomBaseTf.transform.rotation.x = odomBase_.getRotation().x();
            odomBaseTf.transform.rotation.y = odomBase_.getRotation().y();
            odomBaseTf.transform.rotation.z = odomBase_.getRotation().z();
            odomBaseTf.transform.rotation.w = odomBase_.getRotation().w();
            tfPub_.publish(odomBaseTf);
            tfBr_.sendTransform(tf::StampedTransform(odomBase_, imgMsg->header.stamp, odomFrameId_, baseFrameId_));
            
            // Publish dense pointcloud
            if(pubPoints_)
                publishFullPointCloud(depthImg, imgMsg->header.frame_id, imgMsg->header.stamp);
        }

        // If new key-frame or first image
        if(flow > flowThreshold_ || !odomInit_)
        {
            updateKeyframe(rgbImg, depthImg, kpts, desc);
        }

        // Set the initialization to true
        odomInit_ = true;
        //std::cout << "Time: " << t.tock() << std::endl << std::endl;
        //std::cout << std::endl;
    }

    /**
     * @brief Camera calibration info callback
     * @param cInfoMsg Camera calibration message
     */
    void cInfoCallback(const sensor_msgs::CameraInfoConstPtr& cInfoMsg)
    {
        if(!calibInit_)
        {
            // Set calibration flag
            calibInit_ = true;

            // Store RGB camera parameters
            K_ = cv::Mat(3, 3, CV_64FC1, (void *)cInfoMsg->K.elems).clone();
            D_ = cv::Mat(cInfoMsg->D.size(), 1, CV_64FC1, (void *)cInfoMsg->D.data()).clone();
        }
    }


    //cv::FastFeatureDetector fDetector_;         /**< Feature detector (FAST)*/  //OpenCV2
	cv::Ptr<cv::FastFeatureDetector> fDetector_ = cv::FastFeatureDetector::create();   //OpenCV3 


    //cv::BriefDescriptorExtractor fExtractor;  /**< Feature decriptor extractor (BRIEF)*/

	
    //cv::ORB fExtractor_;                        /**< Feature decriptor extractor (ORB)*/  //OpenCV2
	cv::Ptr<cv::ORB> fExtractor_ = cv::ORB::create();	//OpenCV3

    RobustMatcher matcher_;                     /**< Matcher*/

    ros::NodeHandle nh_;                        /**< ROS node handler*/
    image_transport::ImageTransport it_;        /**< Image transport*/

    image_transport::SubscriberFilter imageSub_;    /**< Image subscriber*/
    image_transport::SubscriberFilter depthSub_;    /**< Depth subscriber*/
    ros::Subscriber cInfoSub_;                      /**< Camera info subscriber*/
    message_filters::Synchronizer<syncPolicy> imageSync_;   /**< Time synchronizer filter*/
    image_transport::Publisher imgPub_;             /**< Image publisher*/
    std_msgs::Header imgHeader_;                    /**< Image header for publishing*/
    std::string imgEncodings_;                      /**< Image encodings for publishing*/
    cv::Size imgSize_;                              /**< Image size*/
    ros::Publisher  pcPub_;                         /**< Point cloud publishers*/
    tf::TransformBroadcaster tfBr_;             /**< TF broadcaster*/
    ros::Publisher tfPub_;                      /**< TransformStamped publisher*/
    cv::Mat image_;

    bool odomInit_;         /**< Flag to determine whether to perform odometry or not*/
    bool calibInit_;        /**< Flag indicating if we have calibration data*/
    bool tfCache_;          /**< Flag indicating if we already got the TF from the camera to the base*/

    tf::StampedTransform Tcam2base_;   /**< Transform from camera to base_link*/
    tf::TransformListener tfListener_;

    // Node params
    bool publishImage_;           /**< Flag to determine whether to publish images (OpenCV)*/
    bool pubPoints_;            /**< Flag to determine whether to publish point clouds*/
    int maxFeatures_;           /**< Maximum number of features to find in each bucket*/
    double flowThreshold_;      /**< Minimum feature flow to determine if a new key-frame has to be created*/
    int minMatches_;            /**< Minimum number of matches to perform odometry estimation*/
    std::string odomFrameId_;   /**< Source frame for odometry transformation*/
    std::string baseFrameId_;   /**< Target frame for odometry transformation*/
    int maxKeyframes_;          /**< Maximum number of key-frames to store*/

    cv::Mat K_;                 /**< Camera intrinsic parameters matrix*/
    cv::Mat D_;                 /**< Camera distortion parameters matrix*/

    tf::Transform odom_;        /**< Current odometry*/
    tf::Transform odomK_;       /**< Last key-frame odometry*/
    tf::Transform odomBase_;    /**< Current odometry in base_link frame*/
    KeyFrame kFrame_;           /**< Last key-frame data*/
    
    // Bundle adjustment stuff
    bool useSba_;                   /**< Flag to apply SBA or not*/
    int typeSba_;
    cvsba::Sba sba_;                /**< Sparse Bundle Adjustment class*/
    SbaData matchData_;             /**< Structure with all matched points for SBA*/
    SbaData sbaData_;               /**< Structure with valid data for SBA*/

    // IMU stuff
    bool useImu_;                   /**< Flag to apply IMU corrections or not*/
    ros::Subscriber imuSub_;        /**< IMU data subscriber*/
    bool imuReceived_;              /**< Bool to check whether we truly receive IMU data*/
    double rxImu_, ryImu_;          /**< Roll and pitch angles from IMU data*/
};

#endif
