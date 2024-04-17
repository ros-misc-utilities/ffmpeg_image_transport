# ROS2 image transport for FFmpeg encoding

The ROS2 image transport supports encoding/decoding with the FFMpeg
library, for example encoding h264 and h265 or HEVC, using
Nvidia or other hardware acceleration when available.
This package is a complete rewrite of an
[older ROS1 ffmpeg_image_transport](https://github.com/daniilidis-group/ffmpeg_image_transport)
package.

The publisher plugin of the transport produces 
[ffmpeg image transport messages](https://github.com/ros-misc-utitilies/ffmpeg_image_transport_msgs).
These are raw, encoded packets that are then transmitted and decoded by the
subscriber plugin of the transport. The transport library 
contains both the publisher(encoder) and subscriber(decoder) plugin
and therefore must be installed on both sides to be useful.

To extract e.g. frames or an mp4 file from a recorded bag, have a look at the
[ffmpeg\_image\_transport\_tools](https://github.com/ros-misc-utilities/ffmpeg_image_transport_tools) repository.

## Supported systems

Continuous integration is tested under Ubuntu with the following ROS2 distros:

 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__ffmpeg_image_transport__ubuntu_jammy_amd64&subject=Humble)](https://build.ros2.org/job/Hdev__ffmpeg_image_transport__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Idev__ffmpeg_image_transport__ubuntu_jammy_amd64&subject=Iron)](https://build.ros2.org/job/Idev__ffmpeg_image_transport__ubuntu_jammy_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__ffmpeg_image_transport__ubuntu_jammy_amd64&subject=Rolling)](https://build.ros2.org/job/Rdev__ffmpeg_image_transport__ubuntu_jammy_amd64/)


## Installation

### From packages

```bash
sudo apt-get install ros-${ROS_DISTRO}-ffmpeg-image-transport
```

### From source

Set the following shell variables:
```bash
repo=ffmpeg_image_transport
url=https://github.com/ros-misc-utilities/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

Make sure to source your workspace's ``install/setup.bash`` afterwards.
If all goes well you should see the transport show up:

```
ros2 run image_transport list_transports
```

should give output (among other transport plugins):

```text
"image_transport/ffmpeg"
 - Provided by package: ffmpeg_image_transport
 - Publisher: 
      This plugin encodes frames into ffmpeg compressed packets
    
 - Subscriber: 
      This plugin decodes frames from ffmpeg compressed packets
```

Remember to install the plugin on both hosts, the one that is encoding and
the one that is decoding (viewing).

## Parameters

The plugin has a few parameters that allow for some amount of control.

### Publisher (camera driver)

- ``encoding``: Only ever tested: ``libx264``, ``h264_nvenc``, ``h264``, ``hevc_nvenc``, ``h264_vaapi``.
  If you have an Nvidia card it most likely supports ``hevc_nvenc``.
  This will dramatically reduce the CPU load compare to ``libx264`` (the default).
  You can list all available codecs with ``ffmpeg -codecs``. In the relevant row,
  look for what it says under ``(encoders)``.
- ``preset``: For instance ``slow``, ``ll`` (low latency) etc.
  To find out what presets are available, run e.g.
  ``fmpeg -hide_banner -f lavfi -i nullsrc -c:v libx264 -preset help -f mp4 - 2>&1``
- ``profile``: For instance ``baseline``, ``main``. See [the ffmpeg website](https://trac.ffmpeg.org/wiki/Encode/H.264).
- ``tune``: See [the ffmpeg website](https://trac.ffmpeg.org/wiki/Encode/H.264).
- ``gop_size``: The number of frames inbetween keyframes. Default is ``15``.
  The larger this number the more latency you will have, but also the more efficient
  the transmission becomes.
- ``bit_rate``: The max bit rate [in bits/s] that the encoding will target. Default is ``8242880`.

The parameters are under the ``ffmpeg_image_transport`` variable block. So if you launch
your publisher node (camera driver), you can give it a parameter list on the way like so:
```
        parameters=[{'ffmpeg_image_transport.encoding': 'hevc_nvenc',
                     'ffmpeg_image_transport.profile': 'main',
                     'ffmpeg_image_transport.preset': 'll',
                     'ffmpeg_image_transport.gop': 15}]
```

### Subscriber (viewer)

The subscriber has only one parameter, which is the map between the encoding that was used
to encode the frames, and the decoder to be used for decoding. The mapping is done via parameters.
To tell the subscriber to use the ``hevc`` decoder instead of the default ``hevc_cuvid``
decoder for decoding incoming ``hevc_nvenc`` packets set a parameter like so *after* you started the viewer:
```
ros2 param set <name_of_your_viewer_node> ffmpeg_image_transport.map.hevc_nvenc hevc
```
You also need to refresh the subscription (drop down menu in the viewer) for the parameter to take hold.
If anyone ever figures out how to set the parameters when starting the viewer, please report back.


### Republishing

The ``image_transport`` allows you to republish the decoded image locally,
see for instance [here](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/blob/foxy/README.md).
Here the ROS parameters work as expected to modify the mapping between
encoding and decoder.

The following line shows how to specify the decoder when republishing.
For example to decode incoming ``hevc_nvenc`` packets with the ``hevc`` decoder:
```
ros2 run image_transport republish ffmpeg in/ffmpeg:=image_raw/ffmpeg raw out:=image_raw/uncompressed --ros-args -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"
```

Republishing is generally not necessary so long as publisher and subscriber both properly use
an image transport. Some nodes however, notably the rosbag player, do not support a proper transport,
rendering republishing necessary.

#### Republishing raw images from rosbags in ffmpeg format

Suppose you have raw images in a rosbag but want to play them across a network using
the ``ffmpeg_image_transport``. In this case run a republish node like this
(assuming your rosbag topic is ``/my_camera/image_raw``):
```
ros2 run image_transport republish raw in:=/my_camera/image_raw
```
The republished topic will be under a full transport, meaning you can now view them with e.g. ``rqt_image_view`` under the topic ``/out/ffmpeg``.

You can record them in ``ffmpeg`` format by e.g ``ros2 bag record /out/ffmpeg``.

#### Republishing compressed images from rosbags

Let's say you have stored images as ffmpeg packets in a rosbag under the topic ``/my_camera/ffmpeg``. To view them use this line:
```
ros2 run image_transport republish ffmpeg in/ffmpeg:=/my_camera/ffmpeg raw

```
This will republish the topic with full image transport support.

### Setting encoding parameters when launching camera driver

The ``launch`` directory contains an example launch file ``cam.launch.py`` that demonstrates
how to set encoding profile and preset for e.g. a usb camera.


### How to use a custom version of libav (aka ffmpeg)

Compile *and install* ffmpeg. Let's say the install directory is
``/home/foo/ffmpeg/build``, then for it to be found while building,
run colcon like this:
```
colcon build --symlink-install --cmake-args --no-warn-unused-cli -DFFMPEG_PKGCONFIG=/home/foo/ffmpeg/build/lib/pkgconfig -DCMAKE_BUILD_TYPE=RelWithDebInfo 
```

This will compile against the right headers, but at runtime it may
still load the system ffmpeg libraries. To avoid that, set
``LD_LIBRARY_PATH`` at runtime:
```
export LD_LIBRARY_PATH=/home/foo/ffmpeg/build/lib:${LD_LIBRARY_PATH}
```

## License

This software is issued under the Apache License Version 2.0.
