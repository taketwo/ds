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

#ifndef PCL_IO_DEPTH_SENSE_GRABBER_H
#define PCL_IO_DEPTH_SENSE_GRABBER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/grabber.h>

#include <DepthSense.hxx>

namespace pcl
{

  namespace io
  {
    namespace depth_sense
    {
      class DepthSenseDeviceManager;
    }
  }

  class PCL_EXPORTS DepthSenseGrabber : public Grabber
  {

    public:

      typedef
        void (sig_cb_depth_sense_point_cloud)
          (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);

      enum Mode
      {
        DepthSense_QVGA_30Hz = 0,
      };

      DepthSenseGrabber ();

      virtual
      ~DepthSenseGrabber () throw ();

      virtual void
      start ();

      virtual void
      stop ();

      virtual bool
      isRunning () const;

      virtual std::string
      getName () const
      {
        return (std::string ("DepthSenseGrabber"));
      }

      virtual float
      getFramesPerSecond () const;

      void
      setConfidenceThreshold (int threshold);

    private:

      void
      configureDepthNode (DepthSense::DepthNode node);

      void
      onDepthDataReceived (DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);

      void
      onColorDataReceived (DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
      {
      }

      friend pcl::io::depth_sense::DepthSenseDeviceManager;

      // signals to indicate whether new clouds are available
      boost::signals2::signal<sig_cb_depth_sense_point_cloud>* point_cloud_signal_;

      std::string device_id_;

      bool is_running_;

      int confidence_threshold_;

  };

}

#endif /* PCL_IO_DEPTH_SENSE_GRABBER_H */

