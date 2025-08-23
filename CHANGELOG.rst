^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ffmpeg_image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2025-08-23)
------------------
* fix param dump bug by changing decoder param sep from . to  \_
* Contributors: Bernd Pfrommer

3.0.1 (2025-08-22)
------------------
* improved CI workflow
* Contributors: Bernd Pfrommer

3.0.0 (2025-08-08)
------------------
* support falling back to alternative decoders, rename parameters, add tests etc
  * adopt the new ffmpeg_encoder_api which allows for probing of decoders. The decoder parameter can now contain a list of comma-separated decoders which will be tried in order.
  * adds gtests to the repo
  * reformat to the black python formatter
  * add arguments to some example launch files
  * allow arbitrary AVOptions setting via av_option
  * remove parameters "preset", "tune", "delay", and "crf" (must now be set via "av_options")
  * remove some default values (like bit_rate) to force user to set them explicitly
  * change the parameter names: remove the leading ".", so now the parameters are named specify "camera.image_raw.." as opposed to ".camera.image_raw..."
  * adapt to new ffmpeg_encoder_decoder API
  * change name of parameter from encoding->encoder
  * provide encoding when initializing encoder (new encoder/decoder API)
  * handle new API for image transport 6.3.0
  * deal with Humble bug: topic is passed in without being prefixed by the namespace, but then namespace is removed!
  * package splitting functions into utilities file
* stop building on foxy but still support humble
* Replaced deprecated code
* Contributors: Alejandro Hernandez Cordero, Bernd Pfrommer

2.0.3 (2025-05-26)
------------------
* avoid ament_target_dependencies
* Contributors: Bernd Pfrommer

2.0.2 (2025-03-30)
------------------
* fix bug: segfault when publish function pointer changes
* updated broken badge, fixed typo in readme
* Contributors: Bernd Pfrommer

2.0.1 (2025-03-17)
------------------
* fix bug that prevents compilation in humble
* Contributors: Bernd Pfrommer

2.0.0 (2025-03-15)
------------------
* use ffmpeg_encoder_decoder
* align parameter handling with compressed image transport
* point to new instructions
* Contributors: Bernd Pfrommer

1.0.2 (2025-03-07)
------------------
* updated badges and fixed deprecation warnings (`#36 <https://github.com/ros-misc-utilities/ffmpeg_image_transport/issues/36>`_)
  * updated badges and fixed deprecation warnings
  * only use AV_FRAME_FLAG_KEY when available
* Configurable CRF (`#34 <https://github.com/ros-misc-utilities/ffmpeg_image_transport/issues/34>`_)
  * Added CRF support
  * fixed gop parameter in the README examples
  ---------
  Co-authored-by: Alexey Shtern <alexey.shtern@xtend.me>
* README: Add usage instructions for Jazzy
  The syntax mentioned in the README no longer works there.
* fix typo in link
* added documentation for enabling NVMPI on the jetson
* Contributors: Alexey Shtern, Bernd Pfrommer, Danil Tolkachev, Michal Sojka

1.0.1 (2024-04-17)
------------------
* use appropriate header for cv_bridge
* added documentation and improved cmake exporting
* move encoder->decoder map to decoder for public use
* added frame delay control
* changed install directory so other pkgs can ament_target_depend on this library
* Contributors: Bernd Pfrommer, Toby Buckley

1.0.0 (2024-01-11)
------------------
* initial release of ROS2 package
* Contributors: Akshay Srinivasan, Bernd Pfrommer, Francesco Ferroni, Michal Sojka, akssri
