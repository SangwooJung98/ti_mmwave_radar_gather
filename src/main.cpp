#include <fstream>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <radar_gather/Gather.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "radar_gather_node");
    ros::NodeHandle nh("");

    RadarG fwd_gather(0);
    fwd_gather.ros_init (nh);

    RadarG lower_gather(1);
    lower_gather.ros_init (nh);

    while(ros::ok()) {
        if(!fwd_gather.q.empty())
            fwd_gather.runOnce();
        if(!lower_gather.q.empty())
            lower_gather.runOnce();

        ros::spinOnce();
    }

    return 0;
}