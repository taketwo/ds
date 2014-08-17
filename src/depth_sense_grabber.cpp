/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#include <boost/lexical_cast.hpp>

#include "depth_sense_grabber.h"
#include "depth_sense/depth_sense_device_manager.h"
#include "depth_sense/buffers.h"

using namespace pcl::io::depth_sense;

pcl::DepthSenseGrabber::DepthSenseGrabber (const std::string& device_id)
: Grabber ()
, is_running_ (false)
, confidence_threshold_ (50)
, color_data_ (COLOR_SIZE * 3)
, depth_buffer_ (new SingleBuffer)
{
  if (device_id == "")
    device_id_ = DepthSenseDeviceManager::getInstance ()->captureDevice (this);
  else if (device_id[0] == '#')
    device_id_ = DepthSenseDeviceManager::getInstance ()->captureDevice (this, boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_id_ = DepthSenseDeviceManager::getInstance ()->captureDevice (this, device_id);

  point_cloud_signal_ = createSignal<sig_cb_depth_sense_point_cloud> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_depth_sense_point_cloud_rgba> ();
}

pcl::DepthSenseGrabber::~DepthSenseGrabber () throw ()
{
  stop ();

  DepthSenseDeviceManager::getInstance ()->releaseDevice (device_id_);

  disconnect_all_slots<sig_cb_depth_sense_point_cloud> ();
  disconnect_all_slots<sig_cb_depth_sense_point_cloud_rgba> ();
}

void
pcl::DepthSenseGrabber::start ()
{
  need_xyz_ = num_slots<sig_cb_depth_sense_point_cloud> () > 0;
  need_xyzrgba_ = num_slots<sig_cb_depth_sense_point_cloud_rgba> () > 0;

  if (!is_running_)
  {
    DepthSenseDeviceManager::getInstance ()->reconfigureDevice (device_id_);
    DepthSenseDeviceManager::getInstance ()->startDevice (device_id_);
    frequency_.reset ();
    is_running_ = true;
  }
}

void
pcl::DepthSenseGrabber::stop ()
{
  if (is_running_)
  {
    DepthSenseDeviceManager::getInstance ()->stopDevice (device_id_);
    is_running_ = false;
  }
}

bool
pcl::DepthSenseGrabber::isRunning () const
{
  return (is_running_);
}

float
pcl::DepthSenseGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

void
pcl::DepthSenseGrabber::setConfidenceThreshold (int threshold)
{
  confidence_threshold_ = threshold;
  DepthSenseDeviceManager::getInstance ()->reconfigureDevice (device_id_);
}

void
pcl::DepthSenseGrabber::setDepthIntrinsics (const DepthSense::IntrinsicParameters& intrinsics)
{
  depth_intrinsics_ = intrinsics;

  const float& cx = intrinsics.cx;
  const float& cy = intrinsics.cy;
  const float& fx = intrinsics.fx;
  const float& fy = intrinsics.fy;
  const float& k1 = intrinsics.k1;
  const float& k2 = intrinsics.k2;
  const float& k3 = intrinsics.k3;
  const float& p1 = intrinsics.p1;
  const float& p2 = intrinsics.p2;

  z_to_point_map_.resize (SIZE, 3);

  // Populate a matrix that will be used to map depth values to points coordinates.
  // This code is based `Rectification` class from fovis (https://code.google.com/p/fovis/),
  // which is in turn based on `cvUndistortPoints` from OpenCV.
  for (int y = 0; y < HEIGHT; ++y)
  {
    for (int x = 0; x < WIDTH; ++x)
    {
      // Normalize according to principal point and focal length
      double x1 = (x - cx) / fx;
      double y1 = (y - cy) / fy;
      double x0 = x1;
      double y0 = y1;
      // Iteratively undistort point
      for (int j = 0; j < 5; ++j)
      {
        double r2 = x1 * x1 + y1 * y1;
        double icdist = 1.0 / (1.0 + ((k3 * r2 + k2) * r2 + k1) * r2);
        double delta_x = 2.0 * p1 * x1 * y1 + p2 * (r2 + 2 * x1 * x1);
        double delta_y = p1 * (r2 + 2.0 * y1 * y1) + 2.0 * p2 * x1 * y1;
        x1 = (x0 - delta_x) * icdist;
        y1 = (y0 - delta_y) * icdist;
      }
      z_to_point_map_ (y * WIDTH + x, 0) = x1;
      z_to_point_map_ (y * WIDTH + x, 1) = y1;
      z_to_point_map_ (y * WIDTH + x, 2) = 1.0;
    }
  }
}

void
pcl::DepthSenseGrabber::configureDepthNode (DepthSense::DepthNode node) const
{
  DepthSense::DepthNode::Configuration config = node.getConfiguration ();
  config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
  config.framerate = FRAMERATE;
  config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
  config.saturation = false;
  node.setEnableDepthMapFloatingPoint (true);
  node.setEnableUvMap (true);
  // TODO: deprecated
  node.setConfidenceThreshold (confidence_threshold_);
  node.setConfiguration (config);
}

void
pcl::DepthSenseGrabber::configureColorNode (DepthSense::ColorNode node) const
{
  DepthSense::ColorNode::Configuration config = node.getConfiguration ();
  config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
  config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
  config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
  config.framerate = FRAMERATE;
  node.setEnableColorMap (true);
  node.setConfiguration (config);
  //node.setBrightness(0);
  //node.setContrast(5);
  //node.setSaturation(5);
  //node.setHue(0);
  //node.setGamma(3);
  //node.setWhiteBalance(4650);
  //node.setSharpness(5);
  //node.setWhiteBalanceAuto(true);
}

void
pcl::DepthSenseGrabber::onDepthDataReceived (DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
  fps_mutex_.lock ();
  frequency_.event ();
  fps_mutex_.unlock ();

  static const float nan = std::numeric_limits<float>::quiet_NaN ();

  depth_buffer_->push (data.depthMapFloatingPoint);
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;

  if (need_xyz_)
  {
    xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (WIDTH, HEIGHT));
    xyz_cloud->is_dense = false;

    for (int i = 0; i < SIZE; i++)
    {
      const float& z = (*depth_buffer_)[i];
      if (z == -1.0)
        xyz_cloud->points[i].x = xyz_cloud->points[i].y = xyz_cloud->points[i].z = nan;
      else
        xyz_cloud->points[i].getVector3fMap () = z_to_point_map_.row (i) * z;
    }

    point_cloud_signal_->operator () (xyz_cloud);
  }

  if (need_xyzrgba_)
  {
    xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
    xyzrgba_cloud->is_dense = false;

    for (int i = 0; i < SIZE; i++)
    {
      const float& z = data.depthMapFloatingPoint[i];
      if (z == -1.0)
        xyzrgba_cloud->points[i].x = xyzrgba_cloud->points[i].y = xyzrgba_cloud->points[i].z = nan;
      else
        xyzrgba_cloud->points[i].getVector3fMap () = z_to_point_map_.row (i) * z;

      const DepthSense::UV& uv = data.uvMap[i];
      int row = static_cast<int> (uv.v * COLOR_HEIGHT);
      int col = static_cast<int> (uv.u * COLOR_WIDTH);
      int pixel = row * COLOR_WIDTH + col;
      if (pixel >=0 && pixel < COLOR_WIDTH * COLOR_HEIGHT)
        memcpy (&xyzrgba_cloud->points[i].rgba, &color_data_[pixel * 3], 3);
    }

    point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
  }
}

void
pcl::DepthSenseGrabber::onColorDataReceived (DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
{
  if (need_xyzrgba_)
    memcpy (&color_data_[0], data.colorMap, color_data_.size ());
}

