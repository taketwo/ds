#include <iostream>

#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>

#include "depth_sense_grabber.h"

template <typename PointT>
class DepthSenseProcessor
{

  public:

    typedef pcl::PointCloud<PointT> PointCloudT;

    DepthSenseProcessor (pcl::DepthSenseGrabber& grabber)
    : interface_ (grabber)
    , viewer_ ("DepthSense Cloud Viewer")
    , counter_ (0)
    , timestamp_ (pcl::getTime ())
    , threshold_ (10)
    {
    }

    ~DepthSenseProcessor ()
    {
      connection_.disconnect ();
    }

    void
    run ()
    {
      boost::function<void (const typename PointCloudT::ConstPtr&)> f = boost::bind (&DepthSenseProcessor::cloudCallback, this, _1);
      connection_ = interface_.registerCallback (f);
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
        interface_.setConfidenceThreshold (threshold_);
        threshold_ += 10;
      }
      if (!viewer_.wasStopped ())
        viewer_.showCloud (cloud);
    }

    pcl::DepthSenseGrabber& interface_;
    pcl::visualization::CloudViewer viewer_;
    boost::signals2::connection connection_;
    size_t counter_;
    double timestamp_;

    int threshold_;

};

int main(int argc, const char** argv)
{
  pcl::DepthSenseGrabber grabber;
  DepthSenseProcessor<pcl::PointXYZRGBA> processor (grabber);
  processor.run ();
  return 0;
}

