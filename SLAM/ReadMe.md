# Simultaneous Localisation and Mapping

"The simultaneous localization and mapping (SLAM) problem asks if it is possible for a mobile robot to be placed at an unknown location in an unknown environment and for the robot to incrementally build a consistent map of this environment while simultaneously determining its location within this map. A solution to the SLAM problem has been seen as a “holy grail” for the mobile robotics community as it would provide the means to make a robot truly autonomous."

H. Durrant-Whyte and T. Bailey, "Simultaneous localization and mapping: part I," in IEEE Robotics & Automation Magazine, vol. 13, no. 2, pp. 99-110, June 2006.

# File Description

# Map

The map file is well documented. You insert the resolution that you want to scan at in "res", setup the vertice matrix in the "verts" variable. This should produce a map, and accurately places the agent on the randomly generated position

# MapWithRays

This file introduces the cone width part of the SLAM problem. Disclaimer, this file is unfinished, but provides a decent foundation to start writing SLAM algorithms of your own. Theres also a cone width variable inside of the .m file
