#include <ros/ros.h>
#include <se2_planning/UpdateMapFusion.hpp>

using namespace se2_planning;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_fusion_node");
    ros::NodeHandle nh_;

    TraversabilityMapFusion fuseMap(nh_);

    ros::spin();

    return 0;
}
