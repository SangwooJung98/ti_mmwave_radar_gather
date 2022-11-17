# ti_mmwave_radar_gather
### Radar Gathering Code

This code gathers individual points from each ti_mmwave radar and publish the pointcloud2 msg including x, y, z, vr, az, intensity data. 

## Quick Start Guide

1. Clone this repo onto your   <workspace_dir>/src : 

<pre>
<code>
git clone https://github.com/SangwooJung98/ti_mmwave_radar_gather.git
</code>
</pre>

2. catkin_make and rosrun command to run (need to run roscore preliminary)

<pre>
<code>
cd..
catkin_make
source devel/setup.bash
rosrun radar_gather radar_gather_node
</code>
</pre>

#### TODO
- [ ] topic names into extra configure file for convenience
- [ ] issue of zero intensity (maybe this should be handled in previous driver level....)
