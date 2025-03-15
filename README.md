# ROS2 image transport for ffmpeg/libav

This ROS2 image transport plugin supports encoding/decoding with the FFMpeg
library, for example encoding h264 and h265 or HEVC, using
Nvidia or other hardware acceleration when available.

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
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__ffmpeg_image_transport__ubuntu_noble_amd6464&subject=Jazzy)](https://build.ros2.org/job/Jdev__ffmpeg_image_transport__ubuntu_noble_amd64/)
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

### Publisher (camera driver)

Here is a list of the available encoding parameters:

- ``encoding``: the libav (ffmpeg) encoder being used. The default is ``libx264``, which is on-CPU unaccelerated encoding.
  Depending on your hardware, your encoding options may include the hardware accelerated ``h264_nvenc`` or ``h264_vaapi``.
  You can list all available encoders with ``ffmpeg --codecs``. In the h264 row, look for ``(encoders)``.
- ``preset``: default is empty (""). Valid values can be for instance ``slow``, ``ll`` (low latency) etc.
   To find out what presets are available, run e.g.
   ``fmpeg -hide_banner -f lavfi -i nullsrc -c:v libx264 -preset help -f mp4 - 2>&1``
- ``profile``: For instance ``baseline``, ``main``. See [the ffmpeg website](https://trac.ffmpeg.org/wiki/Encode/H.264).
- ``tune``: See [the ffmpeg website](https://trac.ffmpeg.org/wiki/Encode/H.264). The default is empty("").
- ``gop_size``: The number of frames between keyframes. Default: 10.
   The larger this number the more latency you will have, but also the more efficient the compression becomes.
- ``bit_rate``: The max bit rate [in bits/s] that the encoding will target. Default is ``8242880``.
- ``crf``: Constant Rate Factor, affects the image quality. Value range is ``[0, 51]``; ``0`` is lossless, ``23`` is default, ``51`` is worst quality.
- ``delay``: Not sure what it does, but doesn't help with delay. Default is empty ("").
- ``pixel_format``: Forces a different pixel format for internal conversions. Experimental, don't use.
- ``qmax``: Max quantization rate. Defaults to 10. See [ffmpeg documentation](https://www.ffmpeg.org/ffmpeg-codecs.html).
   The larger this number, the worse the image looks, and the more efficient the encoding.
- ``measure_performance``: For performance debugging (developers only). Defaults to false.
- ``performance_interval``: How many frames to wait between logging performance data.

The parameters are under the ``ffmpeg`` variable block. If you launch
your publisher node (camera driver), you can give it a parameter list on the way like so:
```
        parameters=[{'ffmpeg_image_transport.encoding': 'hevc_nvenc',
                     'ffmpeg_image_transport.profile': 'main',
                     'ffmpeg_image_transport.preset': 'll',
                     'ffmpeg_image_transport.gop_size': 15}]
```
See the example launch file for a V4L USB camera

### Subscriber (viewer)

The subscriber has only one parameter (``map``), which is the map between the encoding that
was used to encode the frames, and the libav decoder to be used for decoding. The mapping is done by creating entries in the ``ffmpeg.map`` parameter, which is prefixed by the image base name, e.g. ``camera``.

For example to tell the subscriber to use the ``hevc`` decoder instead of the default ``hevc_cuvid``
decoder for decoding incoming ``hevc_nvenc`` packets set a parameter like so *after* you started the viewer:
```
ros2 param set <name_of_your_viewer_node> camera.image_raw.ffmpeg.map.hevc_nvenc hevc
```
This is assuming that your viewer node is subscribing to an image ``/camera/image_raw/ffmpeg``.

You also need to refresh the subscription (drop down menu in the viewer) for the parameter to take hold.
If anyone ever figures out how to set the parameters *when* starting the viewer (rather than afterwards!), please update this document.


### Republishing

The ``image_transport`` allows you to republish the decoded image locally,
see for instance [here](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/blob/foxy/README.md).
Here the ROS parameters work as expected to modify the mapping between
encoding and decoder.

The following lines shows how to specify the decoder when republishing.
For example to decode incoming ``hevc_nvenc`` packets with the ``hevc`` decoder:

- ROS 2 Humble:
  ```
  ros2 run image_transport republish ffmpeg in/ffmpeg:=image_raw/ffmpeg raw out:=image_raw/uncompressed --ros-args -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"
  ```
- ROS 2 Jazzy:
  ```
  ros2 run image_transport republish --ros-args -p in_transport:=ffmpeg -p out_transport:=raw --remap in/ffmpeg:=image_raw/ffmpeg --remap out:=image_raw/uncompressed -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"
  ```

Note: The commands below use the Humble syntax and need to be changed as shown here for Jazzy.

Republishing is generally not necessary so long as publisher and subscriber both properly use
an image transport. Some nodes however, notably the rosbag player, do not support a proper transport, rendering republishing necessary.

#### Republishing raw images from rosbags in ffmpeg format

Suppose you have raw images in a rosbag but want to play them across a network using
the ``ffmpeg_image_transport``. In this case run a republish node like this
(assuming your rosbag topic is ``/camera/image_raw``):
```
ros2 run image_transport republish raw in:=/camera/image_raw
```
The republished topic will be under a full transport, meaning you can now view them with e.g. ``rqt_image_view`` under the topic ``/out/ffmpeg``.

You can record them in ``ffmpeg`` format by e.g ``ros2 bag record /out/ffmpeg``.

#### Republishing compressed images from rosbags

Let's say you have stored images as ffmpeg packets in a rosbag under the topic ``/camera/image_raw/ffmpeg``. To view them use this line:
```
ros2 run image_transport republish ffmpeg --ros-args -r in/ffmpeg:=/camera/image_raw/ffmpeg

```
This will republish the topic with full image transport support.

### Setting encoding parameters when launching camera driver

The ``launch`` directory contains an example launch file ``cam.launch.py`` that demonstrates
how to set encoding profile and preset for e.g. a usb camera.


### How to use a custom version of libav (aka ffmpeg)

See the [``ffmpeg_encoder_decoder`` repository](https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder).
There you will also find instructions for hardware accelerated
streaming on the NVidia Jetson.

## License

This software is issued under the Apache License Version 2.0.
