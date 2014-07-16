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

DepthSense::Device
pcl::io::depth_sense::DepthSenseDeviceManager::getDeviceByIndex (size_t index)
{
  // TODO: check if exists
  return (context_.getDevices ().at (index));
}

DepthSense::Device
pcl::io::depth_sense::DepthSenseDeviceManager::getDeviceBySerialNumber (const std::string& sn)
{
  std::vector<DepthSense::Device> devices = context_.getDevices ();
  for (size_t i = 0; i < devices.size (); ++i)
    if (devices[i].getSerialNumber () == sn)
      return (devices[i]);
  THROW_IO_EXCEPTION ("device with serial number %s is not connected", sn.c_str ());
  return (devices[0]);  // never reached, needed just to silence -Wreturn-type warning
}

DepthSense::Device
pcl::io::depth_sense::DepthSenseDeviceManager::getDevice (const std::string& device_id)
{
  throw "not implemented";
}

