/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <velodyne_pointcloud/func.h>

#include <algorithm>
#include <iterator>

#include <opencv2/opencv.hpp>


namespace velodyne_pointcloud
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr extractValidPoints(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr &input_pointcloud, const double min_range, const double max_range)
	{
		pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
    output_pointcloud->reserve(input_pointcloud->points.size());
		for(const auto &p : input_pointcloud->points) {
      if(p.distance >= min_range && p.distance <= max_range) {
				output_pointcloud->points.push_back(p);
			}
		}
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = output_pointcloud->points.size();
		return output_pointcloud;
	}

  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr extractInvalidPoints(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr &input_pointcloud)
	{
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    output_pointcloud->reserve(input_pointcloud->points.size());
    velodyne_pointcloud::PointXYZIR tmp_p;
		for(const auto &p : input_pointcloud->points) {
      if(p.distance == 0 && p.intensity <= 100) {
        tmp_p.x = p.x;
        tmp_p.y = p.y;
        tmp_p.z = p.z;
        tmp_p.intensity = p.intensity;
        tmp_p.ring = p.ring;
				output_pointcloud->points.push_back(tmp_p);
			}
		}
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = output_pointcloud->points.size();
		return output_pointcloud;
	}

  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr extractInvalidNearPoints(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr &input_pointcloud, const std::vector<float> &invalid_intensity_array, const size_t num_lasers)
	{
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    output_pointcloud->reserve(input_pointcloud->points.size());
    velodyne_pointcloud::PointXYZIR tmp_p;
		for(const auto &p : input_pointcloud->points) {
      if(p.distance == 0 && p.intensity <= 100 && p.intensity != invalid_intensity_array[p.ring]) {
        tmp_p.x = p.x;
        tmp_p.y = p.y;
        tmp_p.z = p.z;
        tmp_p.intensity = p.intensity;
        tmp_p.ring = p.ring;
				output_pointcloud->points.push_back(tmp_p);
			}
		}
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = output_pointcloud->points.size();
		return output_pointcloud;
	}

  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr extractInvalidNearPointsFiltered(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr &input_pointcloud, const std::vector<float> &invalid_intensity_array, const size_t num_lasers, const size_t points_size_threshold)
	{

    std::vector<uint16_t> ring_id_array;
    //NOTE support only VLP16 and VLP32C
    if(num_lasers == 16) {
      ring_id_array = { 7, 15, 0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14 };      
    }
    else if(num_lasers == 32) {
      ring_id_array = { 0, 17, 15, 1, 2, 20, 18, 3, 4, 21, 19, 5, 6, 24, 22, 8, 7, 25, 23, 9, 10, 27, 26, 12, 11, 29, 28, 13, 14, 31, 30, 16 };
    }


    velodyne_pointcloud::PointXYZIRADT tmp_p2;
		cv::Mat image = cv::Mat::zeros(cv::Size(input_pointcloud->size()/num_lasers, num_lasers), CV_8UC1);
    for(size_t x = 0; x < image.cols; ++x) {
      for(size_t y = 0; y < image.rows; ++y) {
        tmp_p2 = input_pointcloud->points.at(ring_id_array.at(y)+x*image.rows);
        if(tmp_p2.distance == 0 && tmp_p2.intensity <= 100 && tmp_p2.intensity != invalid_intensity_array[tmp_p2.ring]) {
				  image.at<unsigned char>(y, x) = 255;
        }
        else {
          image.at<unsigned char>(y, x) = 0 ;
        }
			}
		}

  	cv::Mat element(3, 3, CV_8UC1, cv::Scalar::all(255));
		cv::morphologyEx(image, image, cv::MORPH_OPEN, element, cv::Point(-1, -1), 3);
		cv::morphologyEx(image, image, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 3);

		cv::Mat label_image(image.size(), CV_32S);
		cv::Mat stats;
		cv::Mat centroids;
		int label_n = cv::connectedComponentsWithStats(image, label_image, stats, centroids, 8);

		std::vector<int> stat_area;
		for(size_t label = 0; label < label_n; ++label) {
			int* param = stats.ptr<int>(label);
			stat_area.push_back(param[cv::ConnectedComponentsTypes::CC_STAT_AREA]);
			// std::cout << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << " ";
		}
		// std::cout << std::endl;

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    output_pointcloud->reserve(input_pointcloud->points.size());
    int label = 0;
    velodyne_pointcloud::PointXYZIR tmp_p;
    for(size_t x = 0; x < image.cols; ++x) {
		  for(size_t y = 0; y < image.rows; ++y) {
				label = label_image.at<int>(y, x);
				tmp_p2 = input_pointcloud->points.at(ring_id_array.at(y)+x*image.rows);
				if(label != 0 && tmp_p2.intensity != 0 && stat_area.at(label) >= points_size_threshold) {
      		tmp_p.x = tmp_p2.x;
      		tmp_p.y = tmp_p2.y;
      		tmp_p.z = tmp_p2.z;
      		tmp_p.intensity = tmp_p2.intensity;
      		tmp_p.ring = tmp_p2.ring;
				  output_pointcloud->points.push_back(tmp_p);
				}
			}
		}
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = output_pointcloud->points.size();
		return output_pointcloud;
	}

  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr interpotate(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr &input_pointcloud, const std::deque<geometry_msgs::TwistStamped> &twist_queue, const tf2::Transform &tf2_base_link_to_sensor)
	{
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    output_pointcloud->reserve(input_pointcloud->points.size());

    velodyne_pointcloud::PointXYZIR point;
    if(input_pointcloud->points.empty() || twist_queue.empty()) {
      ROS_WARN_STREAM_THROTTLE(10, "input_pointcloud->points or twist_queue is empty.");
      for(const auto& p : input_pointcloud->points) {
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.intensity = p.intensity;
        point.ring = p.ring;
        output_pointcloud->points.push_back(point);
      }
      output_pointcloud->header = input_pointcloud->header;
      output_pointcloud->height = 1;
      output_pointcloud->width = output_pointcloud->points.size();
      return output_pointcloud;
    }

    double theta = 0;
    tf2::Vector3 transed_tf_point(0, 0, 0);

    auto twist_it = std::lower_bound(std::begin(twist_queue), std::end(twist_queue), ros::Time(input_pointcloud->points.front().time_stamp),
      [](const geometry_msgs::TwistStamped &x, ros::Time t) {
        return x.header.stamp < t;
      });
    twist_it = twist_it == std::end(twist_queue) ? std::end(twist_queue)-1 : twist_it;

    for(const auto& p : input_pointcloud->points) {

      for(;(twist_it != std::end(twist_queue)-1 && p.time_stamp > twist_it->header.stamp.toSec()); ++twist_it) {
        // std::cout << std::fixed << p.time_stamp << " " << twist_it->header.stamp.toSec() << std::endl;
      }

      double v = twist_it->twist.linear.x;
      double w = twist_it->twist.angular.z;

      if(std::fabs(p.time_stamp - twist_it->header.stamp.toSec()) > 0.1) {
        ROS_WARN_STREAM_THROTTLE(10, "Twist time_stamp is too late. Cloud not interpotate.");
        v = 0;
        w = 0;
      }

      static double prev_time_stamp = p.time_stamp;
      const double time_offset = p.time_stamp - prev_time_stamp;

      theta += w * time_offset;
      tf2::Quaternion quat;
      quat.setRPY(0.0, 0.0,theta);
      quat = quat*tf2_base_link_to_sensor.getRotation().inverse();

      tf2::Transform trans;
      trans.setOrigin(tf2::Vector3(0, 0, 0));
      trans.setRotation(quat);
      tf2::Vector3 tf_point(v*time_offset, 0, 0);
      transed_tf_point += trans * tf_point;
      point.x = p.x + transed_tf_point.getX();
      point.y = p.y + transed_tf_point.getY();
      point.z = p.z + transed_tf_point.getZ();
      point.intensity = p.intensity;
      point.ring = p.ring;
      output_pointcloud->points.push_back(point);
  
      prev_time_stamp = p.time_stamp;
    }
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = output_pointcloud->points.size();
    return output_pointcloud;
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortRingNumber(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr &input_pointcloud, const size_t num_lasers)
	{
    pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
    output_pointcloud->points.resize(input_pointcloud->points.size());
    pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT> segment_pointcloud;
    for(size_t i = 0; i < input_pointcloud->points.size(); ++i) {

			segment_pointcloud.points.push_back(input_pointcloud->points.at(i));
			if(i % num_lasers == (num_lasers-1)) {
				std::sort(std::begin(segment_pointcloud.points), std::end(segment_pointcloud.points),
				[](const velodyne_pointcloud::PointXYZIRADT &lhs, const velodyne_pointcloud::PointXYZIRADT &rhs){
					return lhs.ring < rhs.ring;
				});
				output_pointcloud->points.insert(std::end(output_pointcloud->points), std::begin(segment_pointcloud.points), std::end(segment_pointcloud.points));
				segment_pointcloud.points.clear();
			}
		}
    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = output_pointcloud->points.size();
    return output_pointcloud;
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr sortZeroIndex(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr& input_pointcloud, const size_t num_lasers)
	{
		size_t zero_index = 0;
		for(size_t i = 0; i < input_pointcloud->points.size(); ++i) {
			if(input_pointcloud->points.at(i).ring == (num_lasers-1)/2){
				static int last_azimuth = input_pointcloud->points.at(i).azimuth;
				if(last_azimuth <= 18000 && input_pointcloud->points.at(i).azimuth > 18000) {
					zero_index = i;
					break;
				}
				last_azimuth = input_pointcloud->points.at(i).azimuth;
			}
		}

    pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
    output_pointcloud->reserve(input_pointcloud->points.size());
		output_pointcloud->points.insert(std::end(output_pointcloud->points), std::begin(input_pointcloud->points)+zero_index, std::end(input_pointcloud->points));
		output_pointcloud->points.insert(std::end(output_pointcloud->points), std::begin(input_pointcloud->points), std::begin(input_pointcloud->points)+zero_index);

    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = output_pointcloud->points.size();
		return output_pointcloud;
	}

  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr convert(const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr &input_pointcloud)
	{
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    output_pointcloud->reserve(input_pointcloud->points.size());
    velodyne_pointcloud::PointXYZIR point;
		for(const auto &p : input_pointcloud->points) {
    		point.x = p.x;
    		point.y = p.y;
    		point.z = p.z;
    		point.intensity = p.intensity;
    		point.ring = p.ring;
				output_pointcloud->points.push_back(point);
		}

    output_pointcloud->header = input_pointcloud->header;
    output_pointcloud->height = 1;
    output_pointcloud->width = output_pointcloud->points.size();
		return output_pointcloud;
	}

}
