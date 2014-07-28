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

#include <boost/thread/mutex.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/grabber.h>

#include <queue>
#include <pcl/common/time.h>

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

  class EventFrequency
  {

    public:

      EventFrequency (size_t window_size = 30)
      : window_size_ (window_size)
      {
        stop_watch_.reset ();
      }

      void event ()
      {
        event_time_queue_.push (stop_watch_.getTimeSeconds ());
        if (event_time_queue_.size () > window_size_)
          event_time_queue_.pop ();
      }

      double
      getFrequency () const
      {
        if (event_time_queue_.size () < 2)
          return (0.0);
        return ((event_time_queue_.size () - 1) /
                (event_time_queue_.back () - event_time_queue_.front ()));
      }

      void reset ()
      {
        stop_watch_.reset ();
        event_time_queue_ = std::queue<double> ();
      }

    private:

      pcl::StopWatch stop_watch_;
      std::queue<double> event_time_queue_;
      const size_t window_size_;

  };

  class PCL_EXPORTS DepthSenseGrabber : public Grabber
  {

    public:

      typedef
        void (sig_cb_depth_sense_point_cloud)
          (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);

      typedef
        void (sig_cb_depth_sense_point_cloud_rgba)
          (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);

      enum Mode
      {
        DepthSense_QVGA_30Hz = 0,
      };

      /** Create a grabber for a DepthSense device.
        *
        * The grabber "captures" the device, making it impossible for other
        * grabbers to interact with it. The device is "released" when the
        * grabber is destructed.
        *
        * This will throw pcl::IOException if there are no free devices that
        * match the supplied \a device_id.
        *
        * \param[in] device_id device identifier, which might be a serial
        * number, an index (with '#' prefix), or an empty string (to select the
        * first available device)
        */
      DepthSenseGrabber (const std::string& device_id = "");

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
      configureDepthNode (DepthSense::DepthNode node) const;

      void
      configureColorNode (DepthSense::ColorNode node) const;

      void
      onDepthDataReceived (DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);

      void
      onColorDataReceived (DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);

      friend pcl::io::depth_sense::DepthSenseDeviceManager;

      // signals to indicate whether new clouds are available
      boost::signals2::signal<sig_cb_depth_sense_point_cloud>* point_cloud_signal_;
      boost::signals2::signal<sig_cb_depth_sense_point_cloud_rgba>* point_cloud_rgba_signal_;

      std::string device_id_;

      bool is_running_;

      int confidence_threshold_;

      bool need_xyz_;
      bool need_xyzrgba_;

      std::vector<uint8_t> color_data_;

      EventFrequency frequency_;
      mutable boost::mutex fps_mutex_;

      static const int FRAMERATE = 30;
      static const int WIDTH = 320;
      static const int HEIGHT = 240;
      static const int SIZE = WIDTH * HEIGHT;
      static const int COLOR_WIDTH = 640;
      static const int COLOR_HEIGHT = 480;
      static const int COLOR_SIZE = COLOR_WIDTH * COLOR_HEIGHT;

  };

}

#endif /* PCL_IO_DEPTH_SENSE_GRABBER_H */

