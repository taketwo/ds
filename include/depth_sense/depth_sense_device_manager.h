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

#ifndef PCL_IO_DEPTH_SENSE_DEVICE_MANAGER_H
#define PCL_IO_DEPTH_SENSE_DEVICE_MANAGER_H

#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <pcl/pcl_exports.h>

#include <DepthSense.hxx>

namespace pcl
{

  class DepthSenseGrabber;

  namespace io
  {

    namespace depth_sense
    {

      class PCL_EXPORTS DepthSenseDeviceManager : boost::noncopyable
      {

        public:

          typedef boost::shared_ptr<DepthSenseDeviceManager> Ptr;

          static Ptr&
          getInstance ()
          {
            static Ptr instance;
            if (!instance)
            {
              boost::mutex::scoped_lock lock (mutex_);
              if (!instance)
                instance.reset (new DepthSenseDeviceManager);
            }
            return (instance);
          }

          inline size_t
          getNumDevices ()
          {
            return (context_.getDevices ().size ());
          }

          std::string
          captureDevice (size_t index, DepthSenseGrabber* grabber);

          std::string
          captureDevice (const std::string& sn, DepthSenseGrabber* grabber);

          void
          releaseDevice (const std::string& sn);

          void
          reconfigureDevice (const std::string& sn);

          void
          startDevice (const std::string& sn);

          void
          stopDevice (const std::string& sn);

        private:

          DepthSenseDeviceManager ();

          std::string
          captureDevice (DepthSense::Device device, DepthSenseGrabber* grabber);

          DepthSense::Context context_;

          static boost::mutex mutex_;

          // thread where the grabbing takes place
          boost::thread depth_sense_thread_;

          size_t active_devices_;

          struct Dev
          {
            DepthSenseGrabber* grabber;
            DepthSense::DepthNode depth_node;
            DepthSense::ColorNode color_node;
          };

          std::map<std::string, Dev> captured_devices_;

      };

    } // namespace depth_sense

  } // namespace io

} // namespace pcl

#endif /* PCL_IO_DEPTH_SENSE_DEVICE_MANAGER_H */

