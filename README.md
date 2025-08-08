# ROS2 image transport for ffmpeg/libav

This ROS2 image transport plugin supports encoding/decoding with the FFMpeg
library, for example encoding h264 and h265/hevc, using
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
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__ffmpeg_image_transport__ubuntu_noble_amd64&subject=Jazzy)](https://build.ros2.org/job/Jdev__ffmpeg_image_transport__ubuntu_noble_amd64/)
 [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kdev__ffmpeg_image_transport__ubuntu_noble_amd64&subject=Kilted)](https://build.ros2.org/job/Kdev__ffmpeg_image_transport__ubuntu_noble_amd64/)
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

Remember to install the plugin on both hosts, the one that is publishing and
the one that is subscribing (viewing).

## Parameters

Below is a short description of the ROS parameters exposed by the Publisher and Subscriber plugins.
The parameters refer to encoder and decoder variables described in more detail in the [``ffmpeg_encoder_decoder`` repository](https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder).

### Publisher (camera driver)

Here is a list of the available encoding parameters:

- ``encoder``: the libav (ffmpeg) encoder being used. The default is ``libx264``, which is on-CPU unaccelerated encoding.
  Depending on your hardware, your encoding options may include the hardware accelerated ``h264_nvenc`` or ``h264_vaapi``.
  You can list all available encoders with ``ffmpeg --codecs``. In the h264 row, look for ``(encoders)``.
- ``encoder_av_options``: default is empty (""). Comma-separeted list of valid
   libav options in the form of (key:value),  e.g.:``'preset:ll,profile:main,crf:0'``.
   See the ffmpeg documentation for [more](https://trac.ffmpeg.org/wiki/Encode/H.264).
- ``gop_size``: The number of frames between keyframes. Default: 10.
   The larger this number the more latency you will have, but also the more efficient the compression becomes.
- ``bit_rate``: The max bit rate [in bits/s] that the encoding will target. Default is ``8242880``.
- ``pixel_format``: Forces a different pixel for encoding.
  This pixel format corresponds to the ``av_source_pixel_format`` in the
  [ffmpeg\_encoder\_decoder documentation](https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder).
- ``qmax``: Max quantization rate. Defaults to 10.
  See [ffmpeg documentation](https://www.ffmpeg.org/ffmpeg-codecs.html).
  The larger this number, the worse the image looks, and the more efficient the encoding.
- ``encoder_measure_performance``: For performance debugging (developers only). Defaults to false.

The parameters are under the ``ffmpeg`` variable block. If you launch
your publisher node (camera driver), you can give it a parameter list on the way like so:
```
        parameters=[{'image_raw.ffmpeg.encoder': 'hevc_nvenc',
                     'image_raw.ffmpeg.encoder_av_options': 'preset:ll,profile:main,crf:0'}]
```
See the example launch file for a V4L USB camera (``usb_camera.launch.py``).
If the above parameter settings don't work, use the ``ros2 param dump <your_node_name>``
command to find out what the proper parameter path is.

### Subscriber (viewer)

- ``decoder_av_options`` comma-separated list of libav options to pass
   to the decoder, like ``foo:bar,foobar:baz``. Default: empty string.
- ``decoder_measure_performance`` enables performance
   measurements. Default: false.
- ``decoders.<codec>.<av_source_pixel_format>.<cv_bridge_target_format>.<original_ros_encoding>`` specifies the decoders to use for a given FFMPEGPacket encoding.
  If no decoders are specified for the full encoding, a less specific parameter will be tested, i.e. ``decoders.<codec>.<av_source_pixel_format>.<cv_bridge_target_format>``, then ``decoders.<codec>.<av_source_pixel_format>``, and finally ``decoders.<codec>``.
  If all parameters are found to be empty, the Subscriber plugin will try to decode with a default set of libav decoders that support the appropriate codec.

All parameters are prefixed again by the image topic and the transport, just like for the publisher node.
The easiest way to find the exact naming of the parameter is to run the node and to use ``ros2 param list`` to find the ``ffmpeg.decoders`` parameter.
Please also refer to the republisher launch script example.

Typically ROS2 parameters are passed to a node by *initializing* the parameters before launching the node, and the node then reads the parameter when it *declares* the parameter.
Unfortunately, this route is not available for the popular ``rqt`` based suite of image viewing tools such as ``rqt_image_view``, since ``rqt`` does not read command line arguments and therefore no parameters can be initialized. The only workaround there is to:
1) start e.g. ``rqt_image_view``
2) select the ffmpeg image transport in the drop down box. This will *declare* the ``decoders`` parameter so you can set it in the next step.
3) find the node name with ``ros2 node list`` and set the parameter using something like
``ros2 param set <name_of_your_viewer_node> camera.image_raw.ffmpeg.decoders.hevc hevc_cuvid``. This assumes your viewer node is subscribing to the topic ``/camera/image_raw/ffmpeg``.
4) again using the drop down box, switch to another image transport and back to ffmpeg. This will cause the ffmpeg transport plugin to apply the updated ``decoders`` parameter.

### Republishing

The ``image_transport`` allows you to republish the decoded image locally,
see for instance [here](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/blob/foxy/README.md).
Here the ROS parameters work as expected to modify the mapping between encoding and decoder.

The following lines shows how to specify the decoder when republishing.
For example to first try to decode incoming ``hevc`` packets with the ``hevc_cuvid`` decoder, and if that fails, with the software ``hevc``decoder:

- ROS 2 Humble:
  ```
  ros2 run image_transport republish ffmpeg in/ffmpeg:=image_raw/ffmpeg raw out:=image_raw/uncompressed --ros-args -p "ffmpeg_image_transport.decoders.hevc:=hevc_cuvid,hevc"
  ```
- ROS 2 Jazzy:
  ```
  ros2 run image_transport republish --ros-args -p in_transport:=ffmpeg -p out_transport:=raw --remap in/ffmpeg:=image_raw/ffmpeg --remap out:=image_raw/uncompressed -p "ffmpeg_image_transport.decoders.hevc:=hevc_cuvid,hevc"
  ```

Note: The commands below use the Humble syntax and need to be changed as shown here for Jazzy.

Republishing is generally not necessary so long as publisher and subscriber both properly use
an image transport.
Some nodes however, notably the rosbag player, do not support a proper transport, making republishing necessary.
Please use the republishing launch file (``republish.launch.py``) in this repo as a starting point for how to set the decoders parameter.

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
ros2 run image_transport republish ffmpeg --ros-args -p in_transport:=ffmpeg -r in/ffmpeg:=/camera/image_raw/ffmpeg
```
This will republish the topic with full image transport support.

### Setting encoding parameters when launching camera driver

The ``launch`` directory contains an example launch file ``usb_camera.launch.py`` that demonstrates
how to set encoding profile and preset for a usb camera.


### How to use a custom version of libav (aka ffmpeg)

See the [``ffmpeg_encoder_decoder`` repository](https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder).
There you will also find instructions for hardware accelerated
streaming on the NVidia Jetson.

## Trouble shooting
### Excessive lag
Many encoders buffer frames to enable inter-frame prediction for improved compression.
This can lead to very large lag. Use a small ``gop_size`` parameter to increase the frequency of keyframes.
### Poor quality
Set the ``bit_rate`` parameter or set the ``crf`` ``av_option`` parameter to something small like  ``crf:1``. Experiment with the ffmpeg CLI tool until you get a satisfying quality.
### Decoder cannot decode image
Check that the ``av_source_pixel_format`` used by the encoder is actually supported by the decoder.
Many decoders can only decode a small number of image formats.


## License

This software is issued under the Apache License Version 2.0.
