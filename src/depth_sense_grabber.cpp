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

using namespace pcl::io::depth_sense;

pcl::DepthSenseGrabber::DepthSenseGrabber (const std::string& device_id)
: Grabber ()
, is_running_ (false)
, confidence_threshold_ (50)
, color_data_ (640 * 480 * 3)
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
  return (0.0f);
}

void
pcl::DepthSenseGrabber::setConfidenceThreshold (int threshold)
{
  confidence_threshold_ = threshold;
  DepthSenseDeviceManager::getInstance ()->reconfigureDevice (device_id_);
}

void
pcl::DepthSenseGrabber::configureDepthNode (DepthSense::DepthNode node) const
{
  DepthSense::DepthNode::Configuration config = node.getConfiguration ();
  config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
  config.framerate = FRAMERATE;
  config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
  config.saturation = false;
  node.setEnableVerticesFloatingPoint (true);
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
  if (need_xyz_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (WIDTH, HEIGHT));
    for (int i = 0; i < SIZE; i++)
    {
      // TODO: how invalid points are represented?
      memcpy (cloud->points[i].data, &data.verticesFloatingPoint[i], 3 * sizeof (float));
    }
    point_cloud_signal_->operator () (cloud);
  }

  if (need_xyzrgba_)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
    for (int i = 0; i < SIZE; i++)
    {
      // TODO: how invalid points are represented?
      memcpy (cloud->points[i].data, &data.verticesFloatingPoint[i], 3 * sizeof (float));

      const DepthSense::UV& uv = data.uvMap[i];
      int row = static_cast<int> (uv.v * 480);
      int col = static_cast<int> (uv.u * 640);
      int pixel = row * 640 + col;
      if (pixel >=0 && pixel < 640 * 480)
        memcpy (&cloud->points[i].rgba, &color_data_[pixel * 3], 3);
    }
    point_cloud_rgba_signal_->operator () (cloud);
  }
}

void
pcl::DepthSenseGrabber::onColorDataReceived (DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
{
  if (need_xyzrgba_)
    memcpy (&color_data_[0], data.colorMap, color_data_.size ());
}

