# PANTHER
**Powerful Autonomous eNTity High-End Robot - Code name: Rude**
##Hardaware repository. All driver packages and installation setup

![Panther robot](https://github.com/rbonghi/panther_hardware/blob/master/panther-nobg.png)

## Project
Panther is a tracked robot with two tracks to explorer the environment outdoor. With the stereocamera [ZED] and the [NVIDIA Jetson TX2] board, this robot can interact with all objects around it. This robot can climbs little rocks and little bumps. it is heavy with 9kg and with the big size 42cm with, 40cm deep and 30cm height, have a ground clearance of 7cm. The tracks have a particular damping system, with three different dampers to absorbe all vibration when the robot drift on the grass. This robot is integrated on [ROS] and all code is available on github. Panther is built with different materials that plexiglass, aluminium, plastic.

**Read more about Panther on http://rnext.it/panther/**

## Develop
* [Installation](http://rnext.it/panther/installation-panther/) - On this guide you can follow the basic step to configure your [NVIDIA Jetson TX2] and unav to work with [ROS].
* [Configuration](http://rnext.it/panther/configuration-panther/) - With the github repository you can configure your robot to run and wandering. In this guide you have all detailed informations to download and set the robot.

## Packages
* **panther_led_controller** - Led controller for panther robot
* **panther_audio** - Audio bridge for ROS

[NVIDIA Jetson TX2]: http://www.nvidia.com/object/embedded-systems-dev-kits-modules.html
[ZED]: https://www.stereolabs.com/
[ROS]: http://www.ros.org/
