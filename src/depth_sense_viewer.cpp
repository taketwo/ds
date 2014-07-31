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

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/io_exception.h>
#include "io_exception.h"

#include "depth_sense_grabber.h"

using namespace pcl::console;

void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "*                       DEPTH SENSE VIEWER - Usage Guide                   *" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << argv[0] << " [Options] device_id" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << std::endl;
  std::cout << "     --help, -h : Show this help"                                             << std::endl;
  std::cout << "     --list, -l : List connected DepthSense devices"                          << std::endl;
  std::cout << "     --xyz      : View XYZ-only clouds"                                       << std::endl;
  std::cout << std::endl;
  std::cout << "Notes:"                                                                       << std::endl;
  std::cout << std::endl;
  std::cout << "   The device to grab data from is selected using device_id argument. It"     << std::endl;
  std::cout << "   could be either:"                                                          << std::endl;
  std::cout << "     * serial number (e.g. YZVF0780239000261D)"                               << std::endl;
  std::cout << "     * device index (e.g. #2 for the second connected device)"                << std::endl;
  std::cout << std::endl;
  std::cout << "   If device_id is not given, then the first available device will be used."  << std::endl;
  std::cout << std::endl;
  std::cout << "   The confidence threshold for depth data can be increased or decreased by"  << std::endl;
  std::cout << "   pressing \"t\" or \"T\" respectively while the focus is on the viewer"     << std::endl;
  std::cout << "   window."                                                                   << std::endl;
}

void
printDeviceList ()
{
  typedef boost::shared_ptr<pcl::DepthSenseGrabber> DepthSenseGrabberPtr;
  std::vector<DepthSenseGrabberPtr> grabbers;
  std::cout << "Connected devices: ";
  boost::format fmt ("\n  #%i  %s");
  while (true)
  {
    try
    {
      grabbers.push_back (DepthSenseGrabberPtr (new pcl::DepthSenseGrabber));
      std::cout << boost::str (fmt % grabbers.size () % grabbers.back ()->getDeviceSerialNumber ());
    }
    catch (pcl::io::IOException& e)
    {
      break;
    }
  }
  if (grabbers.size ())
    std::cout << std::endl;
  else
    std::cout << "none" << std::endl;
}

template <typename PointT>
class DepthSenseViewer
{

  public:

    typedef pcl::PointCloud<PointT> PointCloudT;

    DepthSenseViewer (pcl::DepthSenseGrabber& grabber)
    : grabber_ (grabber)
    , viewer_ ("DepthSense Viewer")
    , threshold_ (50)
    {
      viewer_.registerKeyboardCallback (&DepthSenseViewer::keyboardCallback, *this);
    }

    ~DepthSenseViewer ()
    {
      connection_.disconnect ();
    }

    void
    run ()
    {
      boost::function<void (const typename PointCloudT::ConstPtr&)> f = boost::bind (&DepthSenseViewer::cloudCallback, this, _1);
      connection_ = grabber_.registerCallback (f);
      grabber_.start ();
      while (!viewer_.wasStopped ())
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      grabber_.stop ();
    }

  private:

    void
    cloudCallback (typename PointCloudT::ConstPtr cloud)
    {
      if (!viewer_.wasStopped ())
        viewer_.showCloud (cloud);
    }

    void
    keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if (event.keyDown () && (event.getKeyCode () == 't' || event.getKeyCode () == 'T'))
      {
        if (event.getKeyCode () == 't')
          threshold_ += 10;
        else if (event.getKeyCode () == 'T')
          threshold_ -= 10;

        if (threshold_ < 0)
          threshold_ = 0;

        pcl::console::print_info ("Confidence threshold: ");
        pcl::console::print_value ("%i\n", threshold_);

        grabber_.setConfidenceThreshold (threshold_);
      }
    }

    pcl::DepthSenseGrabber& grabber_;
    pcl::visualization::CloudViewer viewer_;
    boost::signals2::connection connection_;

    int threshold_;

};


int
main (int argc, char** argv)
{
  print_info ("Viewer for DepthSense devices (run with --help for more information)\n", argv[0]);

  if (find_switch (argc, argv, "--help") || find_switch (argc, argv, "-h"))
  {
    printHelp (argc, argv);
    return (0);
  }

  if (find_switch (argc, argv, "--list") || find_switch (argc, argv, "-l"))
  {
    printDeviceList ();
    return (0);
  }

  bool xyz_only = find_switch (argc, argv, "--xyz");

  std::string device_id;

  if (argc == 1 ||             // no arguments
     (argc == 2 && xyz_only))  // single argument, and it is --xyz
  {
    device_id = "";
    print_info ("Creating a grabber for the first available device\n");
  }
  else
  {
    device_id = argv[argc - 1];
    print_info ("Creating a grabber for device \"%s\"\n", device_id.c_str ());
  }

  try
  {
    pcl::DepthSenseGrabber grabber (device_id);
    if (xyz_only)
    {
      DepthSenseViewer<pcl::PointXYZ> viewer (grabber);
      viewer.run ();
    }
    else
    {
      DepthSenseViewer<pcl::PointXYZRGBA> viewer (grabber);
      viewer.run ();
    }
  }
  catch (pcl::io::IOException& e)
  {
    print_error ("Failed to create a grabber: %s\n", e.what ());
    return (1);
  }

  return (0);
}

