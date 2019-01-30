/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cstdint>
#include <cmath>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.h>
//#include "beginner_tutorials/Obstacle.h"
#include <cmath>
#include <queue>
#include <vector>
#include "drivebase/Obstacle.h"

typedef sensor_msgs::PointCloud2 PointCloud;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

/*
void printFrameFormat(const sensor_msgs::PointCloud2ConstPtr& msg) {
	//for (int i = 0; i < msg->fields.size(); i++) {

	//}
	for (auto& it :  msg->fields) {
		ROS_INFO("field %s offset:%d datatype:%d count: %d",
				it.name.c_str(), it.offset, (int)it.datatype, it.count
		);
	}
}*/

struct MyPoint {
	float x;
	float y;
	float z;
	uint8_t empty1[4];
	uint32_t rgb;
	uint8_t empty[12];
};

int framesReceived = 0;
ros::Publisher obs_pub;


const int max_obstacle_count = 3;
const float min_distance_from_robot = 2.00f;
const float min_xy_distance_between_obstacles = 1.0f;

const float min_distance_from_robot_sqr = min_distance_from_robot*min_distance_from_robot;


struct Obstacle2
{
	float distance_sqr;
	float x;
	float y;
	float z;
};

/*template <class T, class S, class C>
S& Container(std::priority_queue<T, S, C>& q) {
    struct HackedQueue : private std::priority_queue<T, S, C> {
        static S& Container(std::priority_queue<T, S, C>& q) {
            return q.*&HackedQueue::c;
        }
    };
    return HackedQueue::Container(q);
}*/

std::vector<Obstacle2> obstaclesv;

void cameraCallback2(const sensor_msgs::PointCloud2ConstPtr& msg) {
	framesReceived++;
	if (framesReceived % 30 == 0) {
		if (msg.get()) {
			ROS_INFO("cameraCallback %d [%dx%d] point_step:%d row_step:%d is_dense:%d stamp:%lld seq:%d frame_id:%s",
					framesReceived,
					(int) msg->width, (int)msg->height,
					(int) msg->point_step, (int)msg->row_step,
					(int) msg->is_dense,
					msg->header.stamp.toNSec(), msg->header.seq, msg->header.frame_id.c_str());

			MyPoint* fp = (MyPoint*)msg->data.data();
			int ystep = (msg->row_step - msg->point_step*msg->width)/msg->point_step;
			//auto cmp = [](Obstacle2& left, Obstacle2& right) { return left.distance_sqr < right.distance_sqr; };
			//std::priority_queue<Obstacle2, std::vector<Obstacle2>, decltype(cmp)> obstacles(cmp);
			//std::vector<Obstacle2> &obstaclesv = Container(obstacles);
			obstaclesv.clear();

			for (int y = 0; y < msg->height; y++, fp += ystep) {
				for (int x = 0; x < msg->width; x++, fp++) // {
					// Some cameras (Kinect) give NAN pixels, so just ignore them
					if (!(std::isnan(fp->x) || std::isnan(fp->y) || std::isnan(fp->z))) {
						// Calculate sqr(distance) from camera. Square root is applied before sending on the network.
						float camera_distance_sqr = fp->x*fp->x + fp->y*fp->y + fp->z*fp->z;
						if (camera_distance_sqr <= 0.0f || camera_distance_sqr > min_distance_from_robot_sqr) {
							continue;
						}

						/*int obstacle_count = obstacles.size();
						if (obstacle_count == 0) {
							//this is the very first obstacle - take it.
							obstacles.push({ camera_distance_sqr, fp->x, fp->y, fp->z});
							continue;
						}
						if (camera_distance_sqr < obstacles.top().distance_sqr) {
							obstacles.push({ camera_distance_sqr, fp->x, fp->y, fp->z });
						}
						if (obstacles.size() > max_obstacle_count) {
							obstacles.pop();
						}*/

						// 1. Find closest point in the array to the current point. Closest is defined only by looking linear distance on x and y axis.
						// 2. Compare closest point distance to camera with current point "fp" distance to camera.
						// 3. If "fp" is closer to camera then: either replace (if points are too close or array full) or add to "obstaclesv" array.
						// 4. if "fp" is not closer to camera, but it is far enough from closet point and there is space in array, add to "obstaclesv".
						// That algorithm does not guarantee at all minimum distance between points is bigger than min_xy_distance_between_obstacles.
						auto closest_point = obstaclesv.begin();
						float closest_xy_distance = 1000000000.0f;
						for (auto it = obstaclesv.begin(); it != obstaclesv.end(); it++) {
							float current_xy_distance = std::fabs(it->x - fp->x) + std::fabs(it->y - fp->y);
							if (current_xy_distance < closest_xy_distance) {
								closest_point = it;
								closest_xy_distance = current_xy_distance;
							}
						}
						if (closest_point != obstaclesv.end()) {
							if (closest_point->distance_sqr > camera_distance_sqr) {
								// if the current point "fp" is closer to camera, we need to either replace or insert it into the array.
								if (closest_xy_distance < min_xy_distance_between_obstacles || obstaclesv.size() >= max_obstacle_count) {
									//if points are too close or array full, then replace closest point with "fp"
									closest_point->x = fp->x;
									closest_point->y = fp->y;
									closest_point->z = fp->z;
									closest_point->distance_sqr = camera_distance_sqr;
									// Note that here we replace the closest_point without checking if every other point in obstaclesv pis
									// at least min_xy_distance_between_obstacles away.
									// TODO: if min_xy_distance_between_obstacles needs to be strictly enforced, then at this point
									// iterate all obstaclesv and find points that are closer than min_xy_distance_between_obstacles.
									// If there are such points, either delete them or delete "this" (depending which one is closer to camera).
								} else {
									obstaclesv.push_back({ camera_distance_sqr, fp->x, fp->y, fp->z});
								}
							} else {
								// if the current point "fp" is further away from camera, but it is far enough from "closest_point"
								// and array is not full, take it
								if (closest_xy_distance > min_xy_distance_between_obstacles && obstaclesv.size() < max_obstacle_count) {
									obstaclesv.push_back({ camera_distance_sqr, fp->x, fp->y, fp->z});
								} else {
									// "fp" is not better - don't use it.
								}
							}
						} else {
							//this is the very first obstacle - take it.
							obstaclesv.push_back({ camera_distance_sqr, fp->x, fp->y, fp->z});
						}
					}
			}

			drivebase::Obstacle omsg;
			omsg.header.stamp = msg->header.stamp;
			omsg.header.frame_id = msg->header.frame_id;
			for (auto& obstacle : obstaclesv) {
				float min_distance = std::sqrt(obstacle.distance_sqr);
				if (min_distance>0) {
					omsg.dx = obstacle.x/min_distance;
					omsg.dy = obstacle.y/min_distance;
					omsg.dz = obstacle.z/min_distance;
					omsg.range = min_distance;
					obs_pub.publish(omsg);
				}
				ROS_INFO("point MIN[%.2fx%.2fx%.2f] %.2f",
						obstacle.x, obstacle.y, obstacle.z, min_distance
						);
			}
		} else {
			ROS_INFO("cameraCallback %d", framesReceived);
		}
	}
}

void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	//cameraCallback 30 [640x480] point_step:32 row_step:20480 is_dense:0
	framesReceived++;
	if (framesReceived % 30 == 0) {
		if (msg.get()) {
			if (30 == framesReceived) {
				//printFrameFormat(msg);
			}
			ROS_INFO("cameraCallback %d [%dx%d] point_step:%d row_step:%d is_dense:%d stamp:%lld seq:%d frame_id:%s",
					framesReceived,
					(int) msg->width, (int)msg->height,
					(int) msg->point_step, (int)msg->row_step,
					(int) msg->is_dense,
					msg->header.stamp.toNSec(), msg->header.seq, msg->header.frame_id.c_str());
			//MyPoint* fp = (MyPoint*)msg->data.data();
			//ROS_INFO("point %.2f %.2f %.2f = 0x%08x", fp->x, fp->y, fp->z, fp->rgb);

			MyPoint* fp = (MyPoint*)msg->data.data();
			int ystep = (msg->row_step - msg->point_step*msg->width)/msg->point_step;
			float x_match = -1.0f, y_match = -1.0f, z_match = -1.0f;
			float x_match2 = -1.0f, y_match2 = -1.0f, z_match2 = -1.0f;
			float min_distance = 0.0f, max_distance = 0.0f;
			bool found = false;
			for (int y = 0; y < msg->height; y++, fp += ystep) {
				for (int x = 0; x < msg->width; x++, fp++) // {
				    if (!(std::isnan(fp->x) || std::isnan(fp->y) || std::isnan(fp->z))) {	
                                        float distance = fp->x*fp->x + fp->y*fp->y + fp->z*fp->z;
					if (distance != 0.0f) {
						if (!found) {
							found = true;
							max_distance = min_distance = distance;
							x_match = fp->x; y_match = fp->y; z_match = fp->z;
							x_match2 = fp->x; y_match2 = fp->y; z_match2 = fp->z;
						}
						if (distance < min_distance) {
							min_distance = distance;
							x_match = fp->x;
							y_match = fp->y;
							z_match = fp->z;
						}
						if (distance > max_distance) {
							max_distance = distance;
							x_match2 = fp->x;
							y_match2 = fp->y;
							z_match2 = fp->z;
						}
					}
				}
			}
			if (found) {
				min_distance = std::sqrt(min_distance);
				max_distance = std::sqrt(max_distance);
			}

			drivebase::Obstacle omsg;
			//msg.header.stamp = ros::Time::now();
			//msg.header.frame_id = "id";
			omsg.header.stamp = msg->header.stamp;
			omsg.header.frame_id = msg->header.frame_id;
		        if (min_distance>0) {	
                        omsg.dx = x_match/min_distance;
			omsg.dy = y_match/min_distance;
			omsg.dz = z_match/min_distance;
			omsg.range = min_distance;
			obs_pub.publish(omsg);
                        }
			ROS_INFO("point MIN[%.2fx%.2fx%.2f] %.2f MAX[%.2fx%.2fx%.2f] %.2f",
					x_match, y_match, z_match, min_distance,
					x_match2, y_match2, z_match2, max_distance
					);
		} else {
			ROS_INFO("cameraCallback %d", framesReceived);
		}
	}
}

/*
void callback(sensor_msgs::ImageConstPtr depth_img_ros){
  cv_bridge::CvImageConstPtr depth_img_cv;
  cv::Mat depth_mat;
  // Get the ROS image to openCV
  depth_img_cv = cv_bridge::toCvShare (depth_img_ros, sensor_msgs::image_encodings::TYPE_16UC1);
  // Convert the uints to floats
  depth_img_cv->image.convertTo(depth_mat, CV_32F, 0.001);
}
 */

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//encoding is TYPE_16UC1
	framesReceived++;
	if (framesReceived % 30 == 0) {
		if (msg.get()) {
			ROS_INFO("depthImageCallback %d [%dx%d] step:%d encoding: %s stamp:%lld seq:%d frame_id:%d",
							framesReceived,
							(int) msg->width, (int)msg->height,
							(int) msg->step, msg->encoding.c_str(),
							msg->header.stamp.toNSec(), msg->header.seq, msg->header.frame_id.c_str());
			uint16_t* fp = (uint16_t*)msg->data.data();
			int ystep = (msg->step - 2*msg->width)/2;
			int x_match = -1, y_match = -1;
			int x_match2 = -1, y_match2 = -1;
			uint16_t depthMM = 0xffff;
			uint16_t maxDepthMM = 0;
			for (int y = 0; y < msg->height; y++, fp += ystep) {
				for (int x = 0; x < msg->width; x++, fp++) {
					if (*fp != 0) {
						if (*fp < depthMM) {
							depthMM = *fp;
							x_match = x;
							y_match = y;
						}
						if (*fp > maxDepthMM) {
							maxDepthMM = *fp;
							x_match2 = x;
							y_match2 = y;
						}
					}
				}
			}

			float depthCM = depthMM / 10;
			float maxdepthCM = maxDepthMM / 10;
			ROS_INFO("point MIN[%dx%d] %.2fcm MAX[%dx%d] %.2fcm", x_match, y_match, depthCM, x_match2, y_match2, maxdepthCM);
		} else {
			ROS_INFO("depthImageCallback %d", framesReceived);
		}
	}
}



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  obs_pub = n.advertise<drivebase::Obstacle>("/obs", 1000);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
// %EndTag(SUBSCRIBER)%

  // topic for realsense "/camera/depth/color/points"
  // topic for kinect "/camera/depth_registered/points"
  ros::Subscriber sub_camera = n.subscribe("/camera/depth_registered/points", 1, cameraCallback2);

  //roslaunch realsense2_camera rs_rgbd.launch
  //ros::Subscriber sub_camera = n.subscribe("/camera/depth/color/points", 1, cameraCallback);

  //roslaunch realsense2_camera rs_camera.launch json_file_path:=/home/stil/Documents/camera2/highdensity.json depth_width:=1280 depth_height:=720 enable_color:=false
  //ros::Subscriber sub_camera = n.subscribe("/camera/depth/image_rect_raw", 10, depthImageCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();

// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

//[ WARN] [1539142335.396453248]: Reconfigure callback failed with exception get_xu(id=11) failed! Last Error: Input/output error:
//Invalid Value in rs2_get_option(options:0x7f2e24005c50, option:Enable Auto Exposure):
//get_xu(id=11) failed! Last Error: Input/output error


/*

File:  drivebase/msg/Obstacle.msg
std_msgs/Header header  # use frame_id to identify the sensor which detected the obstacle
float32 dx              # X component of direction to obstacle in sensor's frame of reference
float32 dy              # Y component of direction to obstacle in sensor's frame of reference
float32 dz              # Z component of direction to obstacle in sensor's frame of reference
float32 range           # distance to the obstacle as measured by the sensor
obs_avoid.py

roslaunch realsense2_camera rs_rgbd.launch json_file_path:=/home/stil/Documents/camera2/highquality.json

 */
