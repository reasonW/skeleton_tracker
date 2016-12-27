/**
 * \ref xtion_tracker.hpp
 *
 *  \date 24/Sep/2016
 *  \author reason_W
 *  \version 1.0
 *  \bug
 *  \copyright GNU Public License.
 */

#ifndef XTION_TRACKER_HPP_
#define XTION_TRACKER_HPP_

// ROS Dependencies
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PointStamped.h>
#include "NiTE.h"
#include "HistoryBuffer.h"
#include <openni2_camera/openni2_device.h>
#include <openni2/OniCTypes.h>
#include <openni2/OpenNI.h>
#include <openni2/OniCEnums.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <xm_msgs/xm_People.h>
#include <xm_msgs/xm_Person.h>
#define MAX_USERS 10

#define USER_MESSAGE(msg) \
        {printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

nite::UserId m_poseUser=0;
uint64_t m_poseTime;
char g_generalMessage[100] = {0};
const int g_poseTimeoutToExit = 2000;
/// Joint map
typedef std::map<std::string, nite::SkeletonJoint> JointMap;
std::map<int, HistoryBuffer<20> *> g_histories;
/**
 * Union for color definition
 */
typedef union
{
  struct
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

/**
 * Class \ref xtion_tracker. This class can track the skeleton of people and returns joints as a TF stream,
 *  while publishing the video stream and the point cloud captured by an ASUS Xtion Pro Live.
 */
class xtion_tracker
{
public:
  /**
   * Constructor
   */
  xtion_tracker() :
      it_(nh_)
  {

    // Get some parameters from the server
    ros::NodeHandle pnh("~");
    if (!pnh.getParam("tf_prefix", tf_prefix_))
    {
      ROS_FATAL("tf_prefix not found on Param Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }
    if (!pnh.getParam("relative_frame", relative_frame_))
    {
      ROS_FATAL("relative_frame not found on Param Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }
    if (!pnh.getParam("camera_frame", camera_frame_))
    {
      ROS_FATAL("camera_frame not found on Parameter Server! Maybe you should add it to your launch file!");
      ros::shutdown();
      return;
    }

    pnh.param("mirror_image", mirrorImage_, false);
    pnh.param("mirror_depth", mirrorDepth_, false);

    // Initialize OpenNI
    if (openni::OpenNI::initialize() != openni::STATUS_OK)
    {
      ROS_FATAL("OpenNI initial error");
      ros::shutdown();
      return;
    }

    // Open the device
    if (devDevice_.open(openni::ANY_DEVICE) != openni::STATUS_OK)
    {
      ROS_FATAL("Can't Open Device");
      ros::shutdown();
      return;
    }
    ROS_INFO("Device opened");

    // Initialize the tracker
    nite::NiTE::initialize();

    // Set the depth mode
    if (depthStream_.create(devDevice_, openni::SENSOR_DEPTH) == openni::STATUS_OK)
    {
      depthMode_ = depthStream_.getSensorInfo().getSupportedVideoModes()[4];
      ROS_INFO("The wished depth mode is %d x %d at %d FPS. Pixel format %d", depthMode_.getResolutionX(),
               depthMode_.getResolutionY(), depthMode_.getFps(), depthMode_.getPixelFormat());
      if (depthStream_.setVideoMode(depthMode_) != openni::STATUS_OK)
      {
        ROS_ERROR("Can't apply depth-videomode");
        depthMode_ = depthStream_.getVideoMode();
        ROS_INFO("The depth mode is set to %d x %d at %d FPS. Pixel format %d", depthMode_.getResolutionX(),
                 depthMode_.getResolutionY(), depthMode_.getFps(), depthMode_.getPixelFormat());
      }

      depthStream_.setMirroringEnabled(mirrorDepth_); // both parameters (depth and color stream) settled as "true", it allows a superposition of video and tracking image not mirrored.
    }
    else
    {
      ROS_FATAL("Can't create depth stream on device");
      ros::shutdown();
      return;
    }

    // Set the video mode
    if (vsColorStream_.create(devDevice_, openni::SENSOR_COLOR) == openni::STATUS_OK)
    {
      // set video mode
      mMode_.setResolution(640, 480);
      mMode_.setFps(30);
      mMode_.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
      ROS_INFO("The wished video mode is %d x %d at %d FPS. Pixel format %d", mMode_.getResolutionX(),
               mMode_.getResolutionY(), mMode_.getFps(), mMode_.getPixelFormat());

      if (vsColorStream_.setVideoMode(mMode_) != openni::STATUS_OK)
      {
        ROS_ERROR("Can't apply videomode\n");
        ROS_INFO("The video mode is set to %d x %d at %d FPS. Pixel format %d", mMode_.getResolutionX(),
                 mMode_.getResolutionY(), mMode_.getFps(), mMode_.getPixelFormat());
        mMode_ = vsColorStream_.getVideoMode();
      }

      // image registration
      if (devDevice_.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
      {
        devDevice_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      }
      vsColorStream_.setMirroringEnabled(mirrorImage_);
    }
    else
    {
      ROS_FATAL("Can't create color stream on device: ");
      ros::shutdown();
      return;
    }

    if (userTracker_.create() != nite::STATUS_OK)
    {
      ROS_FATAL("Couldn't create user tracker");
      ros::shutdown();
      return;
    }
    if (HandTracker_.create() != nite::STATUS_OK)
    {
      ROS_FATAL("Couldn't create hand tracker");
      ros::shutdown();
      return;
    }
    HandTracker_.startGestureDetection(nite::GESTURE_WAVE);
    HandTracker_.startGestureDetection(nite::GESTURE_HAND_RAISE);
    // Start the RGB video stream and the depth video stream
    vsColorStream_.start();
    depthStream_.start();

    // Initialize the image publisher
    imagePub_ = it_.advertise("/camera/rgb/image_raw", 1);

    // Initialize the point cloud publisher
    pointCloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >("/camera/depth_registered/points", 5);

    // Initialize the depth image publisher
    depthPub_ = it_.advertise("/camera/depth_registered/image_raw", 1);

    // Initialize the users IDs publisher
    userPub_ = nh_.advertise<xm_msgs::xm_People>("/people", 1);
    // Initialize both the Camera Info publishers
    depthInfoPub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 1);
    rgbInfoPub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 1);

    rate_ = new ros::Rate(100);

  }
  /**
   * Destructor
   */
  ~xtion_tracker()
  {
    nite::NiTE::shutdown();
  }

  /**
   * Spinner!!!
   */
  void spinner()
  {
    // Broadcast the RGB video
    this->broadcastVideo();

    // If required, publish point_cloud
    if (pointCloudPub_.getNumSubscribers() > 0)
    {
      this->getPointCloud();
    }

    // Broadcast the depth image
    this->getDepth();

    // Broadcast the depth camera info
    depthInfoPub_.publish(this->fillCameraInfo(ros::Time::now(), false));

    // Broadcast the joint frames (if they exist)
    this->getSkeleton();

    rate_->sleep();
  }

private:
  /**
   * RGB Video broadcaster
   */
  void broadcastVideo()
  {

    if (vsColorStream_.readFrame(&vfColorFrame_) == openni::STATUS_OK)
    {
      // convert data to OpenCV format
      const cv::Mat mImageRGB(vfColorFrame_.getHeight(), vfColorFrame_.getWidth(), CV_8UC3,
                              const_cast<void*>(vfColorFrame_.getData()));
      // Check if grabbed frame is actually full with some content
      if (!mImageRGB.empty())
      {
        // Convert the cv image in a ROSy format
        cv::flip(mImageRGB, mImageRGB, 1);
        msg_ = cv_bridge::CvImage(std_msgs::Header(), "rgb8", mImageRGB).toImageMsg();
        msg_->header.frame_id = camera_frame_;
        msg_->header.stamp = ros::Time::now();

        imagePub_.publish(msg_);

        // Publish the rgb camera info
        rgbInfoPub_.publish(this->fillCameraInfo(ros::Time::now(), true));

      }
      else
      {
        ROS_ERROR("Unable to get RGB video");
      }
      vfColorFrame_.release();
    }
    else
    {
      ROS_ERROR("Unable to get RGB video");
    }
  }

  /**
   * Get and publish the point cloud message
   */
  void getPointCloud()
  {
    float centerX, centerY;
    unsigned depthStep = 1, depthSkip = 0;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZRGBA>());
    // Get the depth stream
    depthStream_.start();
    if (depthStream_.readFrame(&depthFrame_) == openni::STATUS_OK)
    {
      unsigned color_step, color_skip;
      // Get some information about the sensor
      openni::DeviceInfo info = devDevice_.getDeviceInfo();
      const char* uri = info.getUri();
      std::string stringa(uri);
      openni2_wrapper::OpenNI2Device dev(stringa);
      // Fill in the message
      cloud_msg->header.stamp = 0;
      cloud_msg->width = depthFrame_.getWidth();
      cloud_msg->height = depthFrame_.getHeight();
      centerX = (cloud_msg->width >> 1) - 0.5f;
      centerY = (cloud_msg->height >> 1) - 0.5f;
      cloud_msg->is_dense = false;
      cloud_msg->points.resize((unsigned long)(cloud_msg->height * cloud_msg->width));
      color_step = 3 * msg_->width / cloud_msg->width;
      color_skip = 3 * (msg_->height / cloud_msg->height - 1) * msg_->width;

      const uint8_t* rgb_buffer = &msg_->data[0];

      // Get the depth data
      const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame_.getData();

      float bad_point = std::numeric_limits<float>::quiet_NaN();

      float constant = 0.001 / dev.getDepthFocalLength(depthFrame_.getHeight());

      cloud_msg->header.frame_id = relative_frame_;

      int color_idx = 0, depth_idx = 0;
      pcl::PointCloud<pcl::PointXYZRGBA>::iterator pt_iter = cloud_msg->begin();
      // Fill in the cloud by merging the depth datat with the color image
      for (int v = 0; v < (int)cloud_msg->height; ++v, color_idx += color_skip)
      {
        for (int u = 0; u < (int)cloud_msg->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
        {
          pcl::PointXYZRGBA& pt = *pt_iter;

          if (pDepth[depth_idx] == 0 || pDepth[depth_idx] > 10000)
          {
            // not valid
            pt.x = pt.y = pt.z = bad_point;
            continue;
          }
          // Fill in XYZRGB
          pt.x = -(u - centerX) * pDepth[depth_idx] * constant;
          pt.y = -(v - centerY) * pDepth[depth_idx] * constant;
          pt.z = pDepth[depth_idx] * 0.001;
          RGBValue color;
          color.Red = rgb_buffer[color_idx];
          color.Green = rgb_buffer[color_idx + 1];
          color.Blue = rgb_buffer[color_idx + 2];
          color.Alpha = 0;
          pt.rgb = color.float_value;
        }
      }
      // Publish to ROS
      sensor_msgs::PointCloud2 pc;
      pcl::toROSMsg(*cloud_msg, pc);
      pc.header.stamp = ros::Time::now();
      pointCloudPub_.publish(pc);
    }

  }

  /**
   * This method publishes the depth image on ROS
   */
  void getDepth()
  {
    depthStream_.start();
    depthStream_.readFrame(&depthFrame_);
    // If the obtained frame is valid, then publish it over  ROS
    if (depthFrame_.isValid())
    {
      openni::DepthPixel* pData = (openni::DepthPixel*)depthFrame_.getData();
      cv::Mat image = cv::Mat(depthStream_.getVideoMode().getResolutionY(),
                              depthStream_.getVideoMode().getResolutionX(),
                              CV_16UC1,
                              pData);

      image.convertTo(image, CV_32FC1, 0.001);
      cv_bridge::CvImage out_msg;
      out_msg.header.stamp = ros::Time::now();
      out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      out_msg.image = image;

      out_msg.header.frame_id = camera_frame_;
      depthPub_.publish(out_msg.toImageMsg());
    }
    else
    {
      ROS_ERROR("Unable to publish depth-image");
    }

  }

  /**
   * Update the Users State
   * @param user: the user
   * @param ts: timestamp
   */
  void updateUserState(const nite::UserData& user, unsigned long long ts)
  {
    if (user.isNew())
      USER_MESSAGE("New")
    else if (user.isVisible() && !g_visibleUsers_[user.getId()])
      USER_MESSAGE("Visible")
    else if (!user.isVisible() && g_visibleUsers_[user.getId()])
      USER_MESSAGE("Out of Scene")
    else if (user.isLost())
      USER_MESSAGE("Lost")

    g_visibleUsers_[user.getId()] = user.isVisible();

    if (g_skeletonStates_[user.getId()] != user.getSkeleton().getState())
    {
      switch (g_skeletonStates_[user.getId()] = user.getSkeleton().getState())
      {
        case nite::SKELETON_NONE:
          USER_MESSAGE("Stopped tracking.")
          break;
        case nite::SKELETON_CALIBRATING:
          USER_MESSAGE("Calibrating...")
          break;
        case nite::SKELETON_TRACKED:
          USER_MESSAGE("Tracking!")
          break;
        case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
        case nite::SKELETON_CALIBRATION_ERROR_HANDS:
        case nite::SKELETON_CALIBRATION_ERROR_LEGS:
        case nite::SKELETON_CALIBRATION_ERROR_HEAD:
        case nite::SKELETON_CALIBRATION_ERROR_TORSO:
          USER_MESSAGE("Calibration Failed... :-|")
          break;
      }
    }
  }

  /**
   * Publish the joints over the TF stream
   * @param j_name: joint name
   * @param j: the joint
   * @param uid: user's ID
   */
  void publishJointTF(std::string j_name, nite::SkeletonJoint j, int uid)
  {

    if (j.getPositionConfidence() > 0.0)
    {
      tf::Transform transform;
      transform.setOrigin(
          tf::Vector3(j.getPosition().x / 1000.0, j.getPosition().y / 1000.0, j.getPosition().z / 1000.0));
      transform.setRotation(tf::Quaternion(0, 0, 0, 1));
      std::stringstream frame_id_stream;
      std::string frame_id;
      frame_id_stream << "/" << tf_prefix_ << "/user_" << uid << "/" << j_name;
      frame_id = frame_id_stream.str();
      tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), relative_frame_, frame_id));
    }
    return;
  }

  /**
   * Get the skeleton's joints and the users IDs
   */
  void getSkeleton()
  {
    if (userTracker_.readFrame(&userTrackerFrame_) != nite::STATUS_OK)
    {
      printf("Get next frame failed\n");
      return;
    }
    if (HandTracker_.readFrame(&handFrame_) != nite::STATUS_OK)
    {
      printf("GetNextData failed\n");
      return;
    }
    // Get all the hands
   nite::UserMap usersMap = userTrackerFrame_.getUserMap();
   std::vector<geometry_msgs::PointStamped> gestures_array;
   std::vector<nite::UserId*> waving_array_Id;
   std::vector<nite::UserId*> raising_array_Id;
   const nite::Array<nite::GestureData>& gestures = handFrame_.getGestures();
   for (int i = 0; i < gestures.getSize(); ++i)
    {
      if (gestures[i].isComplete())
      {
	  geometry_msgs::PointStamped  gestures_tmp;
          const nite::Point3f& position = gestures[i].getCurrentPosition();
          gestures_tmp.point.z=position.x/1000;
          gestures_tmp.point.y=position.y/1000;
          gestures_tmp.point.x=position.z/1000;
          if(gestures[i].getType()==0)
	  {
	     gestures_array.push_back(gestures_tmp);
	     float posX,posY;
	     nite::Status hand2p=HandTracker_.convertHandCoordinatesToDepth(position.x,position.y,position.z,&posX,&posY);
	     if(hand2p==nite::STATUS_OK)
	     {
	         nite::UserId* tmp_Id=(nite::UserId*)((char*)usersMap.getPixels() + ((int)posY*usersMap.getStride()))+(int)posX;
	         waving_array_Id.push_back(tmp_Id);
	     }
          }
	  else if (gestures[i].getType()==2)
	  {
	     float posX,posY;
	     nite::Status hand2p=HandTracker_.convertHandCoordinatesToDepth(position.x,position.y,position.z,&posX,&posY);
	     if(hand2p==nite::STATUS_OK)
	     {
	         nite::UserId* tmp_Id=(nite::UserId*)((char*)usersMap.getPixels() + ((int)posY*usersMap.getStride()))+(int)posX;
	         raising_array_Id.push_back(tmp_Id);
	     }
	  }

          nite::HandId newId;
          HandTracker_.startHandTracking(gestures[i].getCurrentPosition(), &newId);

      }
    }
//    //tracking all hands,add remember their track
//    const nite::Array<nite::HandData>& hands= handFrame_.getHands();
//    for (int i = 0; i < hands.getSize(); ++i)
//    {
//      const nite::HandData& hand = hands[i];
//      if (!hand.isTracking())
//      {
//	 printf("Lost hand %d\n", hand.getId());
//	 nite::HandId id = hand.getId();
//	 HistoryBuffer<20>* pHistory = g_histories[id];
//	 g_histories.erase(g_histories.find(id));
//	 delete pHistory;
//      }
//      else
//      {
//	 if (hand.isNew())
//	 {
//	     printf("Found hand %d\n", hand.getId());
//	     g_histories[hand.getId()] = new HistoryBuffer<20>;
//	 }
//	float posX,posY;
//	nite::Status hand2p=HandTracker_.convertHandCoordinatesToDepth(hands[i].getPosition().x,hands[i].getPosition().y,hands[i].getPosition().z,&posX,&posY);
//	if(hand2p==nite::STATUS_OK)
//	{
//	    nite::UserId* tmp_Id=(nite::UserId*)((char*)usersMap.getPixels() + ((int)posY*usersMap.getStride()))+(int)posX;
//	    waving_array_Id.push_back(tmp_Id);
//	}
//	     // Add to history
//	     HistoryBuffer<20>* pHistory = g_histories[hand.getId()];
//	     pHistory->AddPoint(hand.getPosition());
//      }
//    }
    // Get all the users
    const nite::Array<nite::UserData>& users = userTrackerFrame_.getUsers();
    // Get the skeleton for every user
    xm_msgs::xm_People People_;
    for (int i = 0; i < users.getSize(); ++i)
    {
      xm_msgs::xm_Person Person_;
      const nite::UserData& user = users[i];
      updateUserState(user, userTrackerFrame_.getTimestamp());
      if (user.isNew())
      {
        userTracker_.startSkeletonTracking(user.getId());
	userTracker_.startPoseDetection(user.getId(), nite::POSE_PSI);
	userTracker_.startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
      }
      else
      {
	if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
	{
	    bool hasPose=false;
	    Person_.id=int(user.getId());
	    Person_.posi.point.z=user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().x/1000;
	    Person_.posi.point.y=user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().y/1000;
	    Person_.posi.point.x=user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().z/1000;
	    Person_.posi.header.frame_id="camera_link";
	    //printf("position %d %f %f %f \n", user.getId(),user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().x/1000,user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().y/1000,user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition().z/1000);
	    for(size_t gesture_num=0;gesture_num<raising_array_Id.size();gesture_num++)
	    {
		if(user.getId()==*raising_array_Id[gesture_num] )
		    {
		    	Person_.pose.data="raising";
		    }

      	    }
	   if(!hasPose)
	    {
            for(size_t gesture_num=0;gesture_num<waving_array_Id.size();gesture_num++)
	    {
		if(user.getId()==*waving_array_Id[gesture_num] )
		    {
		    	Person_.pose.data="waving";
			hasPose=true;
		    }
      	    }
	    }
	   if(!hasPose)
	    {
            for(size_t gesture_num=0;gesture_num<gestures_array.size();gesture_num++)
	    {
		double _y_=gestures_array[gesture_num].point.y;
		double _x_=gestures_array[gesture_num].point.z;
		double _z_=gestures_array[gesture_num].point.x;
		double x_=user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x/1000;
		double y_=user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y/1000;
		double z_=user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().z/1000;
 		double disl=sqrt( fabs(_x_ - x_ ))+sqrt(fabs(_y_ - y_))+sqrt( fabs(_z_ - z_) );
		std::cout<<"left hand  "<<x_<<" "<<y_<<" "<<z_<<" "<<std::endl;
		std::cout<<"left distance  "<<disl<<std::endl;
		double _x=user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x/1000;
		double _y=user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y/1000;
		double _z=user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().z/1000;
 		double disr=sqrt(fabs(_x_ -_x) )+sqrt(fabs(_y_ - _y)  )+sqrt(fabs(_z_ - _z) );
		std::cout<<"right hand  "<<_x<<" "<<_y<<" "<<_z<<" "<<std::endl;
		std::cout<<"right distance  "<<disr<<std::endl;
		if(disl<=0.51 || disr<=0.51)
				    Person_.pose.data="waving";
		hasPose=true;
	    }
	    }
	    if(!hasPose)
	    {
		const nite::PoseData& pose1 = user.getPose(nite::POSE_PSI);
	        const nite::PoseData& pose2 = user.getPose(nite::POSE_CROSSED_HANDS);
	        if (pose1.isEntered())
	        {
	            Person_.pose.data="psi";
	        }
	        else if (pose2.isEntered())
	        {
	            Person_.pose.data="cross_hands";
	        }
            }

	 }
        else if(users[i].getSkeleton().getState() ==nite::SKELETON_CALIBRATING)
	 {
	    Person_.id=int(user.getId());
	    Person_.posi.point.z=user.getCenterOfMass().x/1000;
	    Person_.posi.point.y=user.getCenterOfMass().y/1000;
	    Person_.posi.point.x=user.getCenterOfMass().z/1000;
	    Person_.posi.header.frame_id="camera_link";
	    Person_.pose.data="CALIBRATING...";
         }
    People_.person.push_back(Person_);
     }
//        JointMap named_joints;

//        named_joints["head"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
//        named_joints["neck"] = (user.getSkeleton().getJoint(nite::JOINT_NECK));
//        named_joints["left_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
//        named_joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
//        named_joints["left_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
//        named_joints["right_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
//        named_joints["left_hand"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
//        named_joints["right_hand"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));
//        named_joints["torso"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO));
//        named_joints["left_hip"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP));
//        named_joints["right_hip"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP));
//        named_joints["left_knee"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
//        named_joints["right_knee"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
//        named_joints["left_foot"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
//        named_joints["right_foot"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));

//        for (JointMap::iterator it = named_joints.begin(); it != named_joints.end(); ++it)
//        {
//          publishJointTF(it->first, it->second, user.getId());
//        }
        // Add the user's ID
//	if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)


  // Publish the users' IDs
    }
    userPub_.publish(People_);
  }

  /**
   * Method that returns information about the camera
   * @param time: ros timestamp
   * @param is_rgb
   * @return the \ref sensor_msgs::CameraInfoPtr message
   */
  sensor_msgs::CameraInfoPtr fillCameraInfo(ros::Time time, bool is_rgb)
  {

    sensor_msgs::CameraInfoPtr info_msg = boost::make_shared<sensor_msgs::CameraInfo>();
    if (!is_rgb)
    {
      depthStream_.start();
      depthStream_.readFrame(&depthFrame_);
    }

    info_msg->header.stamp = time;
    info_msg->header.frame_id = camera_frame_;
    info_msg->width = is_rgb ? mMode_.getResolutionX() : depthMode_.getResolutionX();
    info_msg->height = is_rgb ? mMode_.getResolutionY() : depthMode_.getResolutionY();
    info_msg->D = std::vector<double>(5, 0.0);
    info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info_msg->K.assign(0.0);
    info_msg->R.assign(0.0);
    info_msg->P.assign(0.0);
    openni::DeviceInfo info = devDevice_.getDeviceInfo();
    const char* uri = info.getUri();
    std::string stringa(uri);
    openni2_wrapper::OpenNI2Device dev(stringa);

    double f =
        is_rgb ? dev.getColorFocalLength(vfColorFrame_.getHeight()) : dev.getColorFocalLength(depthFrame_.getHeight());
    info_msg->K[0] = info_msg->K[4] = f;
    info_msg->K[2] = (info_msg->width / 2) - 0.5;
    info_msg->K[5] = (info_msg->width * 3. / 8.) - 0.5; //aspect ratio for the camera center on kinect and presumably other devices is 4/3
    info_msg->K[8] = 1.0;
    // no rotation: identity
    info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
    // no rotation, no translation => P=K(I|0)=(K|0)
    info_msg->P[0] = info_msg->P[5] = info_msg->K[0];
    info_msg->P[2] = info_msg->K[2];
    info_msg->P[6] = info_msg->K[5];
    info_msg->P[10] = 1.0;
    return (info_msg);
  }

  /// ROS NodeHandle
  ros::NodeHandle nh_;

  bool g_visibleUsers_[MAX_USERS] = {false};
  nite::SkeletonState g_skeletonStates_[MAX_USERS] = {nite::SKELETON_NONE};

  /// Image transport
  image_transport::ImageTransport it_;
  std::string tf_prefix_, relative_frame_, camera_frame_;
  /// Frame broadcaster
  tf::TransformBroadcaster tfBroadcast_;
  /// The openni device
  openni::Device devDevice_;
  /// Some NITE stuff
  nite::UserTracker userTracker_;
  nite::HandTracker HandTracker_;
  nite::Status niteRc_;
  nite::UserTrackerFrameRef userTrackerFrame_;
  nite::HandTrackerFrameRef handFrame_;
  /// Video color stream
  openni::VideoStream vsColorStream_;
  /// OpenNI video mode
  openni::VideoMode mMode_;
  /// OpenNI depth frame reference
  openni::VideoFrameRef depthFrame_;
  /// OpenNI depth stream
  openni::VideoStream depthStream_;
  /// OpenNI depth mode
  openni::VideoMode depthMode_;
  /// OpenNI video frame reference
  openni::VideoFrameRef vfColorFrame_;
  /// RGB image publisher
  image_transport::Publisher imagePub_;
  /// Users IDs publisher
  ros::Publisher userPub_;
  /// Point cloud publisher
  ros::Publisher pointCloudPub_;
  /// Image message
  sensor_msgs::ImagePtr msg_;
  /// Node rate
  ros::Rate* rate_;
  /// Depth image publisher
  image_transport::Publisher depthPub_;

  ///RGB Camera INFO
  sensor_msgs::CameraInfoPtr rgbInfo_;
  ///RGB Camera info publisher
  ros::Publisher rgbInfoPub_;

  /// Depth Info
  sensor_msgs::CameraInfo depthInfo_;
  /// Depth info publisher
  ros::Publisher depthInfoPub_;

  /// Mirroring RGB
  bool mirrorImage_;

  /// Mirroring Depth
  bool mirrorDepth_;

};

#endif /* XTION_TRACKER_HPP_ */
