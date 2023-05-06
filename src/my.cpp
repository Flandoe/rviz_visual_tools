/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Demo implementation of rviz_visual_tools
           To use, add a Rviz Marker Display subscribed to topic /rviz_visual_tools
*/

// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "my_slam/planes_info.h"

// C++
#include <iostream>
#include <algorithm>
#include <iterator>

namespace rvt = rviz_visual_tools;

namespace rviz_visual_tools
{
class RvizVisualToolsDemo
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  rvt::RvizVisualToolsPtr visual_tools_;

  std::string name_;

public:
  std::vector<float> X_old;
  std::vector<float> Y_old;
  std::vector<float> Z_old;
  ros::Subscriber subPlane;
  /**
   * \brief Constructor
   */
  RvizVisualToolsDemo() : name_("rviz_demo")
  {
    visual_tools_.reset(new rvt::RvizVisualTools("map", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting
    subPlane   = nh_.subscribe<my_slam::planes_info> ("my_slam/mapping/global_planes", 20, &RvizVisualToolsDemo::planesHandler, this, ros::TransportHints().tcpNoDelay());

    // ROS_INFO("Sleeping 5 seconds before running demo");
    // ros::Duration(5.0).sleep();

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();
  }

  /**
   * description: plot planes
   */
  void planesHandler(const my_slam::planes_infoConstPtr& msgIn)
  {
    double width_my, length_my, height_my;
    height_my = 0.02;
    if(X_old.empty())
    {    
      std::vector<float> X_v(begin(msgIn->X_planes), end(msgIn->X_planes));
      std::vector<float> Y_v(begin(msgIn->Y_planes), end(msgIn->Y_planes));
      std::vector<float> Z_v(begin(msgIn->Z_planes), end(msgIn->Z_planes));

      width_my = 5; length_my = 25; 
      for (int i = 0;i<X_v.size();i++)
      {
        visual_tools_->publishABCDPlane(0.0, 1.0, 0.0, -X_v[i], rvt::RED, length_my, width_my, height_my);
      }
      width_my = 25; length_my = 5;
      for (int i = 0;i<Y_v.size();i++)
      {
        visual_tools_->publishABCDPlane(1.0, 0.0, 0.0, -Y_v[i], rvt::GREEN, length_my, width_my, height_my);
      }
      width_my = 25; length_my = 25;
      for (int i = 0;i<Z_v.size();i++)
      {
        visual_tools_->publishABCDPlane(0.0, 0.0, 1.0, -Z_v[i], rvt::BLUE, length_my, width_my, height_my);
      }

      X_old = X_v;
      Y_old = Y_v;
      Z_old = Z_v;
    }
    else
    {
      std::vector<float> X_v(begin(msgIn->X_planes), end(msgIn->X_planes));
      std::vector<float> Y_v(begin(msgIn->Y_planes), end(msgIn->Y_planes));
      std::vector<float> Z_v(begin(msgIn->Z_planes), end(msgIn->Z_planes));
      std::vector<float> X_symDifference;
      std::set_symmetric_difference(
        X_v.begin(), X_v.end(),
        X_old.begin(), X_old.end(),
        std::back_inserter(X_symDifference));
      width_my = 5; length_my = 25;    
      for (int i = 0;i<X_symDifference.size();i++)
      {
        visual_tools_->publishABCDPlane(0.0, 1.0, 0.0, -X_symDifference[i], rvt::RED, length_my, width_my, height_my);
      }
      std::vector<float> Y_symDifference;
      std::set_symmetric_difference(
        Y_v.begin(), Y_v.end(),
        Y_old.begin(), Y_old.end(),
        std::back_inserter(Y_symDifference));
      width_my = 25; length_my = 5;
      for (int i = 0;i<Y_symDifference.size();i++)
      {
        visual_tools_->publishABCDPlane(1.0, 0.0, 0.0, -Y_symDifference[i], rvt::GREEN, length_my, width_my, height_my);
      }
      std::vector<float> Z_symDifference;
      std::set_symmetric_difference(
        Z_v.begin(), Z_v.end(),
        Z_old.begin(), Z_old.end(),
        std::back_inserter(Z_symDifference));
      width_my = 25; length_my = 25;
      for (int i = 0;i<Z_symDifference.size();i++)
      {
        visual_tools_->publishABCDPlane(0.0, 0.0, 1.0, -Z_symDifference[i], rvt::BLUE, length_my, width_my, height_my);
      }  
      X_old = X_v;
      Y_old = Y_v;
      Z_old = Z_v;
    }
    visual_tools_->trigger();
  }

  void publishLabelHelper(const Eigen::Isometry3d& pose, const std::string& label)
  {
    Eigen::Isometry3d pose_copy = pose;
    pose_copy.translation().x() -= 0.2;
    visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::XXLARGE, false);
  }

  void testRows(double& x_location)
  {
    // Create pose
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

    pose1.translation().x() = x_location;

    double space_between_rows = 0.2;
    double y = 0;
    double step;

    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying Planes");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.2;
    double max_plane_size = 0.075;
    double min_plane_size = 0.01;
    for (double i = 0; i <= 1.0; i += step)
    {
      visual_tools_->publishXYPlane(pose1, rvt::RED, i * max_plane_size + min_plane_size);
      visual_tools_->publishXZPlane(pose1, rvt::GREEN, i * max_plane_size + min_plane_size);
      visual_tools_->publishYZPlane(pose1, rvt::BLUE, i * max_plane_size + min_plane_size);
      if (i == 0.0)
      {
        publishLabelHelper(pose1, "Planes");
      }

      pose1.translation().x() += step;
    }
    visual_tools_->trigger();


    // --------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED(name_, "Displaying ABCD Plane");
    double x_width = 1.0;
    double y_width = 1.0;

    Eigen::Vector3d n;
    double a, b, c = 0, d;
    y += space_between_rows;
    double x_plane = 0, y_plane = y;

    pose1 = Eigen::Isometry3d::Identity();
    pose1.translation().x() = x_plane;
    pose1.translation().y() = y_plane;
    publishLabelHelper(pose1, "ABCD Plane");

    // for (std::size_t i = 0; i < 10; ++i)
    // {
    //   x_plane = i * step;
    //   a = x_plane;
    //   b = y_plane;
    //   // D takes this value to satisfy Ax+By+D=0
    //   d = -(x_plane * x_plane + y_plane * y_plane);
    //   visual_tools_->publishABCDPlane(a, b, c, d, rvt::MAGENTA, x_width, y_width);
    //   x_location += step;
    // }
    // visual_tools_->publishABCDPlane(0.997397, -0.0720573, 0.00263565, 15.0638892548455, rvt::MAGENTA, x_width, y_width);
    // visual_tools_->publishABCXYZPlane(0.997397, -0.0720573, 0.00263565, -14.303960, 10.915579, -0.558252, rvt::MAGENTA, 10, 10);
    // visual_tools_->publishABCXYZPlane(0.999056, -0.0413521, -0.0132835, -12.227930, 12.720082, -0.045222, rvt::MAGENTA, 10, 10);
    // visual_tools_->publishABCXYZPlane(0.998654, -0.0489393, -0.0171942, -13.358409, 10.575362, -0.478792, rvt::MAGENTA, 10, 10);
    // visual_tools_->publishABCXYZPlane(0.0389379, 0.998547, 0.0372547, -13.625232, 13.652045, -0.365729, rvt::MAGENTA, 10, 10);
    
    // visual_tools_->publishABCXYZPlane(0.678832 , 0.734179 , -0.0129817, -15.813153, 6.545786, -0.602630, rvt::RED, 10, 10);
    // visual_tools_->publishABCXYZPlane(0.65166 , 0.758039 , -0.0267445, -14.911167, 7.007274, -0.668613, rvt::RED, 10, 10);
    // visual_tools_->publishABCXYZPlane(0.759979 , -0.649283 , -0.0293799, -17.001572, 8.339624, -0.524730, rvt::RED, 10, 10);
    // visual_tools_->publishABCXYZPlane(0.633811 , 0.772973 , -0.0282195, -15.539643, 8.905488, -0.304419, rvt::RED, 10, 10);
    // // visual_tools_->publishABCXYZPlane(0.717488 , -0.696443 , -0.0133388, -14.241579, 5.497253, 9.145461, rvt::RED, 10, 10,0.01);
    // visual_tools_->publishABCXYZPlane(0.75093 , -0.66038 , -0.00151955, -14.065008, 5.638114, 6.117486, rvt::RED, 10, 10,0.01);
    double width_my = 5;
    double length_my = 5;
    double height_my = 0.02;
    visual_tools_->publishABCXYZPlane(0.6604 , 0.7509 , 0.0, -15.813153, 6.545786, -0.602630, rvt::RED, length_my,width_my, height_my);
    visual_tools_->publishABCXYZPlane(0.6604 , 0.7509 , 0.0, -14.911167, 7.007274, -0.668613, rvt::RED, length_my,width_my, height_my);
    visual_tools_->publishABCXYZPlane(0.75093 , -0.66038 , 0.0, -17.001572, 8.339624, -0.524730, rvt::RED, length_my,width_my, height_my);
    visual_tools_->publishABCXYZPlane(0.6604 , 0.7509 , 0.0, -15.539643, 8.905488, -0.304419, rvt::RED, length_my,width_my, height_my);
    visual_tools_->publishABCXYZPlane(0.75093 , -0.66038 , 0.0, -14.065008, 5.638114, 6.117486, rvt::RED, length_my,width_my, height_my);
    visual_tools_->publishABCXYZPlane(0.75093 , -0.66038 , 0.0, -13.434724,6.922389,1.822765, rvt::RED, length_my,width_my, height_my);
    visual_tools_->publishABCXYZPlane(0.75093 , -0.66038 , 0.0, -14.160015,6.039485,19.208918, rvt::GREEN, length_my,width_my, height_my);
    visual_tools_->publishABCXYZPlane(0.0 , 0.0 , 1.0, -19.012680, 0.872816, 20.134787, rvt::RED, length_my,width_my, height_my);
    // Set x location for next visualization function
    x_location += 1.25;

    visual_tools_->trigger();
  }

  /** \brief Compare sizes of markers using all MEDIUM-scale markers */
  void testSize(double& x_location, scales scale)
  {
    // Create pose
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

    // Reusable vector of 2 colors
    std::vector<colors> colors;
    colors.push_back(RED);
    colors.push_back(GREEN);

    // Reusable points vector
    EigenSTL::vector_Vector3d points1;
    EigenSTL::vector_Vector3d points2;

    double step = 0.25;  // space between each row

    // Show test label
    pose1.translation().x() = x_location - 0.1;
    visual_tools_->publishText(pose1, "Testing consistency of " + visual_tools_->scaleToString(scale) + " marker scale",
                               WHITE, XLARGE, false);

    pose1.translation().x() = x_location;

    // TODO(dave): publishCone() - no scale version available
    // TODO(dave): publishXYPlane() - no scale version available
    // TODO(dave): publishXZPlane() - no scale version available
    // TODO(dave): publishYZPlane() - no scale version available

    // Sphere
    visual_tools_->publishSphere(pose1, BLUE, scale);
    pose1.translation().y() += step;

    // Spheres
    points1.clear();
    points1.emplace_back(pose1.translation());
    pose1.translation().x() += step;
    points1.emplace_back(pose1.translation());
    visual_tools_->publishSpheres(points1, BLUE, scale);
    pose1.translation().x() = x_location;  // reset
    pose1.translation().y() += step;

    // Spheres with colors
    points1.clear();
    points1.emplace_back(pose1.translation());
    pose1.translation().x() += step;
    points1.emplace_back(pose1.translation());
    visual_tools_->publishSpheres(points1, colors, scale);
    pose1.translation().x() = x_location;  // reset
    pose1.translation().y() += step;

    // YArrow
    visual_tools_->publishYArrow(pose1, BLUE, scale);
    pose1.translation().y() += step;

    // ZArrow
    visual_tools_->publishZArrow(pose1, GREEN, scale);
    pose1.translation().y() += step;

    // XArrow
    visual_tools_->publishXArrow(pose1, RED, scale);
    pose1.translation().y() += step;

    // Arrow (x arrow)
    visual_tools_->publishArrow(pose1, RED, scale);
    pose1.translation().y() += step;

    // Line
    pose2 = pose1;
    pose2.translation().x() += step / 2.0;
    visual_tools_->publishLine(pose1, pose2, PURPLE, scale);
    pose1.translation().y() += step;

    // Lines
    points1.clear();
    points2.clear();
    pose2 = pose1;
    pose2.translation().x() += step / 2.0;
    points1.emplace_back(pose1.translation());
    points2.emplace_back(pose2.translation());
    pose1.translation().x() += step / 2.0;

    pose2 = pose1;
    pose2.translation().x() += step / 2.0;
    // points1.push_back(pose1.translation());
    // points2.push_back(pose2.translation());
    colors.clear();  // temp
    colors.push_back(ORANGE);
    visual_tools_->publishLines(points1, points2, colors, scale);
    pose1.translation().x() = x_location;  // reset
    pose1.translation().y() += step;

    // TODO(dave): publishPath
    // TODO(dave): publishPolygon
    // TODO(dave): publishWireframeCuboid
    // TODO(dave): publishWireframeRectangle

    // Axis Labeled
    visual_tools_->publishAxisLabeled(pose1, "Axis", scale);
    pose1.translation().y() += step;

    // Axis
    visual_tools_->publishAxis(pose1, scale);
    pose1.translation().y() += step;

    // TODO(dave): publishAxis

    // Cylinder
    pose2 = pose1;
    pose2.translation().x() += step / 2.0;
    visual_tools_->publishCylinder(pose1.translation(), pose2.translation(), BLUE, scale);
    pose1.translation().y() += step;

    // TODO(dave): publishMesh

    // TODO(dave): publishGraph

    // Text
    visual_tools_->publishText(pose1, "Text", WHITE, scale, false);
    pose1.translation().y() += step;

    // Display test
    visual_tools_->trigger();

    // Set x location for next visualization function
    x_location += 0.5;
  }

  /** \brief Compare every size range */
  void testSizes(double& x_location)
  {
    ROS_INFO_STREAM_NAMED(name_, "Testing sizes of marker scale");

    // Create pose
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

    // Show test label
    pose1.translation().x() = x_location - 0.1;
    visual_tools_->publishText(pose1, "Testing sizes of marker scale", WHITE, XLARGE, false);

    pose1.translation().x() = x_location;
    pose2.translation().x() = x_location;

    // Sphere
    for (scales scale = XXXXSMALL; scale <= XXXXLARGE; /*inline*/)
    {
      if (scale == MEDIUM)
      {
        visual_tools_->publishSphere(pose1, GREEN, scale);
      }
      else
      {
        visual_tools_->publishSphere(pose1, GREY, scale);
      }
      visual_tools_->publishText(pose2, "Size " + visual_tools_->scaleToString(scale), WHITE, scale, false);

      scale = static_cast<scales>(static_cast<int>(scale) + 1);
      pose1.translation().y() += visual_tools_->getScale(scale).x + 0.1;

      // Text location
      pose2.translation().y() = pose1.translation().y();
      pose2.translation().x() = x_location + visual_tools_->getScale(scale).x * 1.3;
    }

    // Display test
    visual_tools_->trigger();

    // Set x location for next visualization function
    x_location += 0.5;
  }
};  // end class

}  // namespace rviz_visual_tools

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_demo");
  ROS_INFO_STREAM("Visual Tools Demo");


  rviz_visual_tools::RvizVisualToolsDemo demo;

  double x_location = 0;
  demo.testRows(x_location);
//   demo.testSize(x_location, rviz_visual_tools::MEDIUM);
//   demo.testSize(x_location, rviz_visual_tools::LARGE);
//   demo.testSizes(x_location);

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
