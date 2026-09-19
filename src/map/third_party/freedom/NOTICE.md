FreeDOM vendor notice
=====================

Source: https://github.com/LC-Robotics/FreeDOM
Commit: dcfd5690cafddf121a80bd8ea477807d9656748e
License: MIT, copied in LICENSE.

Vendored files:
- include/freedom/common_types.h
- include/freedom/depth_image.h
- include/freedom/freedom.h
- include/freedom/map.h
- include/freedom/mrmap.h
- include/freedom/raycast.h
- include/freedom/scanmap.h
- include/freedom/utils.h
- src/depth_image.cpp
- src/freedom.cpp
- src/map.cpp
- src/mrmap.cpp
- src/raycast.cpp
- src/scanmap.cpp

Local portability changes:
- Removed ROS1-only includes and helper glue from freedom.h, scanmap.h, and utils.h.
- Replaced ROS time use in ProgressBar with std::chrono.
- Silenced Timer per-stage stdout and destructor summaries so the ROS2 node controls logging.
- Added standard library includes that were previously pulled in indirectly by ROS headers.
- Guarded empty worker chunks in scanmap.cpp to avoid invalid point-cloud iterator arithmetic.
- Initialized DepthImage::learn_fov to false so destruction is safe when raycast enhancement is disabled.
- Used defined unsigned multiplication in IndexHash to avoid signed overflow as map coordinates grow.
