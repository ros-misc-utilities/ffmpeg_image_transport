# ROS2 image transport for FFmpeg encoding

The ROS2 image transport supports encoding/decoding with the FFMpeg
library, for example encoding h264 and h265 or HEVC, using
Nvidia or other hardware acceleration when available.
This package is a complete rewrite of an
[older ROS1 ffmpeg_image_transport](https://github.com/daniilidis-group/ffmpeg_image_transport)
package.

The publisher plugin of the transport produces 
[ffmpeg image transport messages](https://github.com/ros-misc-utilities/ffmpeg_image_transport_msgs/).
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
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__ffmpeg_image_transport__ubuntu_noble_amd64&subject=Jazzy)](https://build.ros2.org/job/Jdev__ffmpeg_image_transport__ubuntu_noble_amd64/)
[![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__ffmpeg_image_transport__ubuntu_noble_amd64&subject=Rolling)](https://build.ros2.org/job/Rdev__ffmpeg_image_transport__ubuntu_noble_amd64/)


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

There are various ROS parameters to control the encoding. They are described
in the [``ffmpeg_encoder_decoder`` repository](https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder?tab=readme-ov-file#publisher-camera-driver).

The parameters are under the ``ffmpeg_image_transport`` variable block. So if you launch
your publisher node (camera driver), you can give it a parameter list on the way like so:
```
        parameters=[{'ffmpeg_image_transport.encoding': 'hevc_nvenc',
                     'ffmpeg_image_transport.profile': 'main',
                     'ffmpeg_image_transport.preset': 'll',
                     'ffmpeg_image_transport.gop': 15}]
```

### Subscriber (viewer)

The subscriber has only one parameter (``map``), which is the map between the encoding that
was used to encode the frames, and the libav decoder to be used for decoding. The mapping is done by creating entries in the ``ffmpeg_image_transport.map`` parameter.
To tell the subscriber to use the ``hevc`` decoder instead of the default ``hevc_cuvid``
decoder for decoding incoming ``hevc_nvenc`` packets set a parameter like so *after* you started the viewer:
```
ros2 param set <name_of_your_viewer_node> ffmpeg_image_transport.map.hevc_nvenc hevc
```
You also need to refresh the subscription (drop down menu in the viewer) for the parameter to take hold.
If anyone ever figures out how to set the parameters *when* starting the viewer (rather than afterwards!), please update this document.


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

### How to use ffmpeg hardware accelerated encoding on the NVidia Jetson

Follow the instructions
[here](https://github.com/jocover/jetson-ffmpeg) to build a version of
ffmpeg that supports NVMPI. Then follow the section above on how to
actually use that custom ffmpeg library. As always first test on the
CLI that the newly compiled ``ffmpeg`` command now supports
``h264_nvmpi``. The transport can now be configured to use
nvmpi like so:

```
        parameters=[{'ffmpeg_image_transport.encoding': 'h264_nvmpi',
                     'ffmpeg_image_transport.profile': 'main',
                     'ffmpeg_image_transport.preset': 'll',
                     'ffmpeg_image_transport.gop': 15}]
```


## License

This software is issued under the Apache License Version 2.0.
