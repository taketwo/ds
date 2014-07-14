#include <iostream>

#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>

#include "depth_sense_grabber.h"

template <typename PointT>
class DepthSenseProcessor
{

  public:

    typedef pcl::PointCloud<PointT> PointCloudT;

    DepthSenseProcessor (pcl::Grabber& grabber)
    : interface_ (grabber)
    , viewer_ ("DepthSense Cloud Viewer")
    , counter_ (0)
    , timestamp_ (pcl::getTime ())
    {
    }

    void
    run ()
    {
      boost::function<void (const typename PointCloudT::ConstPtr&)> f = boost::bind (&DepthSenseProcessor::cloudCallback, this, _1);
      interface_.registerCallback (f);
      interface_.start ();
      while (!viewer_.wasStopped ())
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      interface_.stop ();
    }

  private:

    void
    cloudCallback (typename PointCloudT::ConstPtr cloud)
    {
      if (++counter_ == 30)
      {
        double now = pcl::getTime ();
        std::cout << "Average framerate: " << double (counter_) / double (now - timestamp_) << " Hz" << std::endl;
        counter_ = 0;
        timestamp_ = now;
      }
      if (!viewer_.wasStopped ())
        viewer_.showCloud (cloud);
    }

    pcl::Grabber& interface_;
    pcl::visualization::CloudViewer viewer_;
    size_t counter_;
    double timestamp_;

};

int main(int argc, const char** argv)
{
  pcl::DepthSenseGrabber grabber;
  DepthSenseProcessor<pcl::PointXYZ> processor (grabber);
  processor.run ();
  return 0;
}

