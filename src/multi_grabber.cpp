#include <iostream>

#include <boost/shared_ptr.hpp>

#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/io_exception.h>
#include "io_exception.h"

#include "depth_sense_grabber.h"

typedef boost::shared_ptr<pcl::DepthSenseGrabber> DepthSenseGrabberPtr;

template <typename PointT>
class DepthSenseViewer
{

  public:

    typedef pcl::PointCloud<PointT> PointCloudT;

    DepthSenseViewer ()
    : viewer_ ("DepthSense Viewer")
    {
    }

    ~DepthSenseViewer ()
    {
      for (size_t i = 0; i < connections_.size (); ++i)
        connections_[i].disconnect ();
    }

    void
    addGrabber (const DepthSenseGrabberPtr& grabber)
    {
      std::string sn = grabber->getDeviceSerialNumber ();
      boost::function<void (const typename PointCloudT::ConstPtr&)> f = boost::bind (&DepthSenseViewer::cloudCallback, this, _1, sn);
      connections_.push_back (grabber->registerCallback (f));
      grabbers_.push_back (grabber);
      std::cout << "Adding device with serial number: " << sn << std::endl;
      typename PointCloudT::Ptr dummy (new PointCloudT);
      viewer_.addPointCloud (dummy, sn);
      grabber->start ();
    }

    void
    run ()
    {
      while (!viewer_.wasStopped ())
        viewer_.spinOnce (10);
      for (size_t i = 0; i < grabbers_.size (); ++i)
        grabbers_[i]->stop ();
    }

  private:

    void
    cloudCallback (typename PointCloudT::ConstPtr cloud, const std::string& name)
    {
      if (!viewer_.wasStopped ())
        viewer_.updatePointCloud (cloud, name);
    }

    pcl::visualization::PCLVisualizer viewer_;
    std::vector<boost::shared_ptr<pcl::DepthSenseGrabber> > grabbers_;
    std::vector<boost::signals2::connection> connections_;

};

int
main (int argc, const char** argv)
{
  DepthSenseViewer<pcl::PointXYZRGBA> viewer;

  while (true)
  {
    try
    {
      boost::shared_ptr<pcl::DepthSenseGrabber> grabber (new pcl::DepthSenseGrabber);
      viewer.addGrabber (grabber);
    }
    catch (pcl::io::IOException& e)
    {
      break;
    }
  }
  viewer.run ();

  return 0;
}

