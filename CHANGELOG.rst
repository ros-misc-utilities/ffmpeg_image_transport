^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ffmpeg_image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
