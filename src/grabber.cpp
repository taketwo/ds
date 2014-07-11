#include <iostream>
#include <ctime>

#include "depth_sense_grabber.h"

int main(int argc, const char** argv)
{
  pcl::DepthSenseGrabber grabber;
  std::cout << "Starting grabber" << std::endl;
  grabber.start ();
  clock_t start = clock ();
  std::cout << "Waiting " << start << std::endl;
  while (clock () < start + CLOCKS_PER_SEC * 2);
  std::cout << "Stopping grabber" << std::endl;
  grabber.stop ();
  return 0;
}

