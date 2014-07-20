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

#include <pcl/io/io_exception.h>

#include "depth_sense/depth_sense_device_manager.h"
#include "depth_sense_grabber.h"

boost::mutex pcl::io::depth_sense::DepthSenseDeviceManager::mutex_;

pcl::io::depth_sense::DepthSenseDeviceManager::DepthSenseDeviceManager ()
: active_devices_ (0)
{
  try
  {
    context_ = DepthSense::Context::create ("localhost");
  }
  catch (...)  // TODO: catch only specific exceptions?
  {
    THROW_IO_EXCEPTION ("failed to initialize DepthSense context");
  }
}

std::string
pcl::io::depth_sense::DepthSenseDeviceManager::captureDevice (size_t index, DepthSenseGrabber* grabber)
{
  if (index >= context_.getDevices ().size ())
    THROW_IO_EXCEPTION ("device with index %i is not connected", index);
  return (captureDevice (context_.getDevices ().at (index), grabber));
}

std::string
pcl::io::depth_sense::DepthSenseDeviceManager::captureDevice (const std::string& sn, DepthSenseGrabber* grabber)
{
  std::vector<DepthSense::Device> devices = context_.getDevices ();
  for (size_t i = 0; i < devices.size (); ++i)
  {
    if (devices[i].getSerialNumber () == sn)
    {
      if (captured_devices_.count (sn))
        THROW_IO_EXCEPTION ("device with serial number %s is captured by another DepthSenseGrabber", sn.c_str ());
      else
        return (captureDevice (devices[i], grabber));
    }
  }
  THROW_IO_EXCEPTION ("device with serial number %s is not connected", sn.c_str ());
  return ("");  // never reached, needed just to silence -Wreturn-type warning
}

void
pcl::io::depth_sense::DepthSenseDeviceManager::reconfigureDevice (const std::string& sn)
{
  const Dev& dev = captured_devices_[sn];
  context_.requestControl (dev.depth_node, 0);
  dev.grabber->configureDepthNode (dev.depth_node);
  context_.releaseControl (dev.depth_node);
  //{
    //context_.requestControl (captured_devices_[sn].color_node, 0);
    //grabber->configureDepthNode (captured_devices_[sn].color_node);
  //}
}

void
pcl::io::depth_sense::DepthSenseDeviceManager::startDevice (const std::string& sn)
{
  const Dev& dev = captured_devices_[sn];
  context_.registerNode (dev.depth_node);
  //context_.registerNode (dev.color_node);
  context_.startNodes ();
  boost::mutex::scoped_lock lock (mutex_);
  if (active_devices_++ == 0)
  {
    std::cout << "first active device: starting thread" << std::endl;
    depth_sense_thread_ = boost::thread (&DepthSense::Context::run, &context_);
  }

}

void
pcl::io::depth_sense::DepthSenseDeviceManager::stopDevice (const std::string& sn)
{
  const Dev& dev = captured_devices_[sn];
  context_.unregisterNode (dev.depth_node);
  //context_.unregisterNode (dev.color_node);
  boost::mutex::scoped_lock lock (mutex_);
  // use getRegisteredNodes ().size ();
  if (active_devices_ == 0)
    return;
  if (--active_devices_ == 0)
  {
    std::cout << "last active device: stopping thread" << std::endl;
    context_.stopNodes ();
    context_.quit ();
    depth_sense_thread_.join ();
  }
}

void
pcl::io::depth_sense::DepthSenseDeviceManager::releaseDevice (const std::string& sn)
{
  captured_devices_.erase (sn);
}

std::string
pcl::io::depth_sense::DepthSenseDeviceManager::captureDevice (DepthSense::Device device, DepthSenseGrabber* grabber)
{
  // TODO: sync?
  Dev dev;
  dev.grabber = grabber;
  std::vector<DepthSense::Node> nodes = device.getNodes ();
  for (size_t i = 0; i < nodes.size (); ++i)
  {
    if (nodes[i].is<DepthSense::DepthNode> ())
    {
      dev.depth_node = nodes[i].as<DepthSense::DepthNode> ();
      dev.depth_node.newSampleReceivedEvent ().connect (grabber, &DepthSenseGrabber::onDepthDataReceived);
    }
    if (nodes[i].is<DepthSense::ColorNode> ())
    {
      dev.color_node = nodes[i].as<DepthSense::ColorNode> ();
      dev.color_node.newSampleReceivedEvent ().connect (grabber, &DepthSenseGrabber::onColorDataReceived);
    }
  }
  captured_devices_.insert (std::make_pair (device.getSerialNumber (), dev));
  return (device.getSerialNumber ());
}

