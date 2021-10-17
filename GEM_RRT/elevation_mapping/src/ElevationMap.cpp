/*
 * ElevationMap.cpp
 *
 *  Created on: Dec 22, 2019
 *      Author: Peter XU
 *	 Institute: ZJU, CSC 104
 */

#include "elevation_mapping/ElevationMap.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

// Grid Map
#include <grid_map_msgs/GridMap.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

using namespace std;
using namespace grid_map;


namespace elevation_mapping {

ElevationMap::ElevationMap(ros::NodeHandle nodeHandle, string robot_name)
    : nodeHandle_(nodeHandle),
      rawMap_({"elevation", "min_height", "height", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", "color", "timestamp", "time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan"}),
      visualMap_({"elevation", "variance", "rough", "slope", "traver", "color_r", "color_g", "color_b", "intensity", "frontier"}),
      robot_name_(robot_name),
      hasUnderlyingMap_(false),
      visibilityCleanupDuration_(0.0)
{
  rawMap_.setBasicLayers({"elevation", "variance"});
  visualMap_.setBasicLayers({"elevation"});

  nodeHandle_.param("orthomosaic_saving_dir", orthoDir, string("/home/mav-lav/Datasets/zjg_image/"));

  clear();

  visualMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/" + robot_name + "/visual_map", 1);
  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/" + robot_name + "/elevation_map_raw", 1);
  VpointsPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/" + robot_name + "/visualpoints",1);
  orthomosaicPublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/" + robot_name + "/orthomosaic", 1);
}

ElevationMap::~ElevationMap()
{
}


void ElevationMap::setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position)
{
  rawMap_.setGeometry(length, resolution, position);
  visualMap_.setGeometry(length, resolution, position);
}


sensor_msgs::ImagePtr ElevationMap::show(ros::Time timeStamp, string robot_name, float trackPointTransformed_x, float trackPointTransformed_y, int length, float *elevation, 
                                          float *var, int *point_colorR, int *point_colorG, int *point_colorB, float *rough, float *slope, float *traver, float* intensity)
{
  cv::Mat image(length, length, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat floodImage(length, length, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat frontierImage(length, length, CV_8UC3, cv::Scalar(0,0,0));

  visualMap_.clearAll();
  std::vector<int> tmpRow;

  tmpMap.clear();
  floodMap.clear();
  dilatedMap.clear();

  for(int i = 0; i < length; i++){
    for(int j = 0; j < length; j++){
      tmpRow.push_back(-1);
    }
    tmpMap.push_back(tmpRow);
    floodMap.push_back(tmpRow);
    dilatedMap.push_back(tmpRow);
  }

  pcl::PointCloud<pcl::PointXYZRGB> show_pointCloud;
  pcl::PointXYZRGB show_point;

  int index, index_x, index_y;
  Index start_index = visualMap_.getStartIndex();

  for (GridMapIterator iterator(visualMap_); !iterator.isPastEnd(); ++iterator) {
    index_x = (*iterator).transpose().x();
    index_y = (*iterator).transpose().y();
    index = index_x * length + index_y;
    if(elevation[index] != -10)
    {
      visualMap_.at("elevation", *iterator) = elevation[index];
      visualMap_.at("variance", *iterator) = var[index];
      visualMap_.at("rough", *iterator) = rough[index];
      visualMap_.at("slope", *iterator) = slope[index];
      visualMap_.at("traver", *iterator) = traver[index];
      visualMap_.at("color_r", *iterator) = point_colorR[index];
      visualMap_.at("color_g", *iterator) = point_colorG[index];
      visualMap_.at("color_b", *iterator) = point_colorB[index];
      visualMap_.at("intensity", *iterator) = intensity[index];
      visualMap_.at("frontier", *iterator) = 0;

      if(traver[index] > 0.7){
        tmpMap[(index_x + length - start_index[0]) % length][(index_y + length - start_index[1]) % length] = 0;
        dilatedMap[(index_x + length - start_index[0]) % length][(index_y + length - start_index[1]) % length] = 0;
      }else{
        tmpMap[(index_x + length - start_index[0]) % length][(index_y + length - start_index[1]) % length] = 1;
        dilatedMap[(index_x + length - start_index[0]) % length][(index_y + length - start_index[1]) % length] = 1;
      }

      Position point;
      visualMap_.getPosition(*iterator, point);

      show_point.x = point.x();
      show_point.y = point.y();
      show_point.z = visualMap_.at("elevation", *iterator);
      show_point.r = visualMap_.at("color_r", *iterator);
      show_point.g = visualMap_.at("color_g", *iterator);
      show_point.b = visualMap_.at("color_b", *iterator);

      show_pointCloud.push_back(show_point);
      image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[0] = visualMap_.at("color_b", *iterator);
      image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[1] = visualMap_.at("color_g", *iterator);
      image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[2] = visualMap_.at("color_r", *iterator);
    }
  }

  for(int i = 0; i < length; i++){
    for(int j = 0; j < length; j++){
      int count = 0;
      for(int k = 0; k < 8; k++){
        int xx = i + dx[k];
        int yy = j + dy[k];
        if(tmpMap[i][j] == 1 && xx >= 0 && yy >= 1 && xx < length && yy < length)
          dilatedMap[xx][yy] = 1;
      }
      if(dilatedMap[i][j] == -1){
        floodImage.at<cv::Vec3b>(i, j)[0] = dilatedMap[i][j] * 128;
        floodImage.at<cv::Vec3b>(i, j)[1] = dilatedMap[i][j] * 128;
        floodImage.at<cv::Vec3b>(i, j)[2] = dilatedMap[i][j] * 128;
      }else{
        floodImage.at<cv::Vec3b>(i, j)[0] = dilatedMap[i][j] * 255;
        floodImage.at<cv::Vec3b>(i, j)[1] = dilatedMap[i][j] * 255;
        floodImage.at<cv::Vec3b>(i, j)[2] = dilatedMap[i][j] * 255;
      }

    }
  }

  // Publish orthomoasic image
  sensor_msgs::ImagePtr fmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", floodImage).toImageMsg();
  orthomosaicPublisher_.publish(fmsg);

  static int count = 0;
  std::ostringstream strs_x, strs_y, cc;
  strs_x << trackPointTransformed_x;
  strs_y << trackPointTransformed_y;
  cc << count;
  std::string str = orthoDir + cc.str()  + ".jpg";
  cv::imwrite(str, image);
  count++;

  pcl_conversions::toPCL(ros::Time::now(), show_pointCloud.header.stamp);
  show_pointCloud.header.frame_id = "odom";

  if(show_pointCloud.size() > 0)
  {
    sensor_msgs::PointCloud2 pub_pointcloud;
    pcl::toROSMsg(show_pointCloud, pub_pointcloud);
    VpointsPublisher_.publish(pub_pointcloud); 
  }

  grid_map::GridMap floodMap_;
  // floodMap_ = findFrontiers(visualMap_, length);
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(visualMap_, message);
  visualMapPublisher_.publish(message);
  return fmsg;
}


void ElevationMap::dfs(vector<std::vector<int>>& map, int x, int y, int length)
{
  if(map[x][y] == 0){
    floodMap[x][y] = 0;

    for(int j = 0; j < 4; j++){
      int xx = x + dx[j];
      int yy = y + dy[j];
      if(xx >= 0 && yy >= 0 && xx < length && yy < length){
        if(map[xx][yy] == 0 && floodMap[xx][yy] == -1){
          dfs(map, xx, yy, length);
        }else if(map[xx][yy] == -1 || (floodMap[xx][yy] == -1 && (xx == 0 || yy == 0))){
          floodMap[x][y] = 2;
        }      
      }
    }
  }

}


grid_map::GridMap ElevationMap::findFrontiers(grid_map::GridMap& gridMap, int length) 
{
  grid_map::Index position;
  grid_map::Index startIndex = gridMap.getStartIndex();

  for(int i = 0; i < length; i++){
    for(int j = 0; j < length; j++){
      int count = 0;
      if(tmpMap[i][j] == -1){
        tmpMap[i][j] = 0;
        dilatedMap[i][j] = 0;
      }

      for(int k = 0; k < 8; k++){
        int xx = i + dx[k];
        int yy = j + dy[k];

        if(tmpMap[i][j] == 0 && xx >= 0 && yy >= 0 && xx < length && yy < length){
          if(tmpMap[xx][yy] == -1)
            dilatedMap[xx][yy] = 0;
        }
        if(tmpMap[i][j] == 1 && xx >= 0 && yy >= 0 && xx < length && yy < length){
          dilatedMap[xx][yy] = 1;
        }
      }
    }
  }

  dfs(dilatedMap, length/2, length/2, length);

  for(int i = 0; i < length; i++){
    for(int j = 0; j < length; j++){
      if(floodMap[i][j] == 2){
        position.x() = (i + startIndex[0]) % length;
        position.y() = (j + startIndex[1]) % length;
        gridMap.at("frontier", position) = 1;
      }else{
        position.x() = (i + startIndex[0]) % length;
        position.y() = (j + startIndex[1]) % length;
        gridMap.at("frontier", position) = 0;
      }
    }
  }
  return gridMap;
}


bool ElevationMap::clear()
{
  rawMap_.clearAll();
  rawMap_.resetTimestamp();
  visualMap_.clearAll();
  visualMap_.resetTimestamp();
  return true;
}

void ElevationMap::opt_move(Position M_position, float update_height)
{
  visualMap_.setPosition(M_position); 

  for (GridMapIterator iterator(visualMap_); !iterator.isPastEnd(); ++iterator) {
    if(visualMap_.at("elevation", *iterator) != -10)
    {
      visualMap_.at("elevation", *iterator) += update_height;
    }
  }
}

void ElevationMap::move(const Index M_startindex, Position M_position)
{
  visualMap_.setStartIndex(M_startindex);
  visualMap_.setPosition(M_position);
}

grid_map::GridMap& ElevationMap::getRawGridMap()
{
  return rawMap_;
}

ros::Time ElevationMap::getTimeOfLastUpdate()
{
  return ros::Time().fromNSec(rawMap_.getTimestamp());
}

const kindr::HomTransformQuatD& ElevationMap::getPose()
{
  return pose_;
}

void ElevationMap::setFrameId(const std::string& frameId)
{
  rawMap_.setFrameId(frameId);
  visualMap_.setFrameId(frameId);
}

const std::string& ElevationMap::getFrameId()
{
  return rawMap_.getFrameId();
}


} /* namespace */
