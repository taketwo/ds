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

#include "depth_sense/depth_sense_device_manager.h"
#include "depth_sense_grabber.h"

pcl::DepthSenseGrabber::DepthSenseGrabber ()
: Grabber ()
, is_running_ (false)
{
  using namespace pcl::io::depth_sense;
  DepthSenseDeviceManager::Ptr manager = DepthSenseDeviceManager::getInstance ();
  std::cout << "There are " << manager->getDevices ().size () << " devices connected" << std::endl;
  auto device = manager->getDeviceBySerialNumber ("YZVF0780251000095M");
  std::vector<DepthSense::Node> nodes = device.getNodes ();
  std::cout << "Found " << nodes.size () << " nodes" << std::endl;
  for (size_t i = 0; i < nodes.size (); ++i)
    if (nodes[i].is<DepthSense::DepthNode> ())
      depth_node_ = nodes[i].as<DepthSense::DepthNode> ();

  point_cloud_signal_ = createSignal<sig_cb_depth_sense_point_cloud> ();
}

pcl::DepthSenseGrabber::~DepthSenseGrabber () throw ()
{
  if (is_running_)
    stop ();

  // TODO: don't forget to update this with new callbacks
  disconnect_all_slots<sig_cb_depth_sense_point_cloud> ();
}

void
pcl::DepthSenseGrabber::start ()
{
  if (is_running_)
    return;

  if (depth_node_.isSet ())
  {
    configureDepthNode ();
    pcl::io::depth_sense::DepthSenseDeviceManager::getInstance ()->registerNode (depth_node_);
  }
  pcl::io::depth_sense::DepthSenseDeviceManager::getInstance ()->startDevice ();
  is_running_ = true;
}

void
pcl::DepthSenseGrabber::stop ()
{
  if (depth_node_.isSet ())
    pcl::io::depth_sense::DepthSenseDeviceManager::getInstance ()->unregisterNode (depth_node_);

  pcl::io::depth_sense::DepthSenseDeviceManager::getInstance ()->stopDevice ();
  is_running_ = false;
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
pcl::DepthSenseGrabber::configureDepthNode ()
{
  DepthSense::DepthNode::Configuration config = depth_node_.getConfiguration ();
  config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
  config.framerate = 30;
  config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
  config.saturation = true;
  depth_node_.setEnableVertices (true);
  depth_node_.newSampleReceivedEvent ().connect (this, &pcl::DepthSenseGrabber::onDepthDataReceived);
  pcl::io::depth_sense::DepthSenseDeviceManager::getInstance ()->configureNode (depth_node_, config);
}

void
pcl::DepthSenseGrabber::onDepthDataReceived (DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
  if (num_slots<sig_cb_depth_sense_point_cloud> () > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> (320, 240));
    for (int i = 0; i < 76800; i++)
    {
      //if (data.vertices[i].z > 100 || data.vertices[i].z < 2000)
      //{
        //cloud->points[i] = pcl::PointXYZ (0, 0, 0);
        //continue;
      //}
      cloud->points[i].x = data.vertices[i].x;
      cloud->points[i].y = data.vertices[i].y;
      cloud->points[i].z = data.vertices[i].z;
    }
    point_cloud_signal_->operator () (cloud);
  }
}

