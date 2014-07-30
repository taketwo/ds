Installation
============

1. Clone this repository:

   ```bash
   git clone https://github.com/taketwo/ds.git ds
   cd ds
   ```

2. [Download](http://www.softkinetic.com/support/download.aspx) and install the
   DepthSense SDK for Linux.

3. Copy the contents of `third-party/compatibility` to the `lib` directory of
   the SDK installation. For example, assuming that SDK was installed in
   `/opt/softkinetic/DepthSenseSDK` (default), do:

   ```bash
   sudo cp third-party/compatibility/* /opt/softkinetic/DepthSenseSDK/lib/
   ```

4. Configure and build this project:

   ```bash
   mkdir build
   cd build
   cmake ..
   make depth_sense_viewer
   ```

Depth Sense Viewer
==================

Connect a DepthSense camera and run the viewer:

```bash
./depth_sense_viewer
```

![Depth Sense Viewer](images/screenshot.png)

Press `t`/`T` to increase/decrease the confidence threshold.
