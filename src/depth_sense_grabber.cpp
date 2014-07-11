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

#include "depth_sense_grabber.h"

pcl::DepthSenseGrabber::DepthSenseGrabber ()
: Grabber ()
{
  point_cloud_signal_ = createSignal<sig_cb_depth_sense_point_cloud> ();
}

pcl::DepthSenseGrabber::~DepthSenseGrabber () throw ()
{
  stop ();
  // TODO: don't forget to update this with new callbacks
  disconnect_all_slots<sig_cb_depth_sense_point_cloud> ();
}

void
pcl::DepthSenseGrabber::start ()
{
  // TODO: may throw if another grabber created context
  context_ = DepthSense::Context::create ("localhost");

  std::vector<DepthSense::Device> devices = context_.getDevices ();
  if (devices.size ())
  {
    std::vector<DepthSense::Node> nodes = devices[0].getNodes ();
    std::cout << "Found " << nodes.size () << " nodes" << std::endl;
    for (size_t i = 0; i < nodes.size (); ++i)
    {
      if (nodes[i].is<DepthSense::DepthNode> ())
      {
        depth_node_ = nodes[i].as<DepthSense::DepthNode> ();
        configureDepthNode ();
        context_.registerNode (depth_node_);
      }
    }
  }

  context_.startNodes ();

  grabber_thread_ = boost::thread (&DepthSense::Context::run, &context_);
}

void
pcl::DepthSenseGrabber::stop ()
{
  context_.stopNodes ();
  if (depth_node_.isSet ())
    context_.unregisterNode (depth_node_);
  // TODO: figure out how to terminate properly
  context_.quit ();
  //grabber_thread_.interrupt ();
  grabber_thread_.join ();
}

bool
pcl::DepthSenseGrabber::isRunning () const
{
  return (false);
}

std::string
pcl::DepthSenseGrabber::getName () const
{
  return (std::string ("DepthSenseGrabber"));
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
  depth_node_.setEnableVerticesFloatingPoint (true);
  context_.requestControl (depth_node_, 0);
  depth_node_.setConfiguration (config);
  depth_node_.newSampleReceivedEvent ().connect (this, &pcl::DepthSenseGrabber::onDepthDataReceived);
}

void
pcl::DepthSenseGrabber::onDepthDataReceived (DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
  std::cout << "Depth data arrived" << std::endl;
}

