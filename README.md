:tiger2: Panther
------------
**P**owerful **A**utonomous e**NT**ity **H**igh-**E**nd **R**obot

Panther is an outdoor tracked robot, with a ZED stereocamera and an NVIDIA Jetson [AGX Xavier], this robot can interact with all objects around it.

This robot can climbs little rocks and little bumps. it is heavy with 11kg and with the big size 42cm with, 40cm deep and 30cm height, have a ground clearance of 7cm. The tracks have a particular damping system, with three different dampers to absorbe all vibration when the robot drift on the grass.

Read more about [Panther]

## Packages
* **panter_drivers** Configuration and startup panther hardware drivers
  - Roboteq [SDC2130]
  - RPLidar [A2]
  - Stereolabs [ZED2]
* **panther_joystick** - Interface to enable feature for panther
* **panther_led_controller** - Led controller for panther robot

[Panther]: http://rnext.it/panther/
[AGX Xavier]: https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit
[TX2]: http://www.nvidia.com/object/embedded-systems-dev-kits-modules.html
[SDC2130]: https://www.roboteq.com/index.php/roboteq-products-and-services/brushed-dc-motor-controllers/249/sdc21xx-family
[Stereolabs]: https://www.stereolabs.com/
[ZED2]: https://www.stereolabs.com/zed-2/
[A2]: https://www.slamtec.com/en/Lidar/A2
[ROS]: http://www.ros.org/
[DCDC-USB]: https://www.mini-box.com/DCDC-USB?sc=8&category=981
