
#pragma once
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <std_srvs/SetBool.h>
#include <se2_navigation_msgs/RequestNavigationMapSrv.h>

using namespace grid_map;
namespace se2_planning {


class TraversabilityMapFusion
{
public:

    TraversabilityMapFusion(ros::NodeHandle& nh);
    ~TraversabilityMapFusion();

    bool loadInitialTraversabilityMap();
    bool loadLocalTraversabilityMap();
    void initialTraversabilityMapCallback(const grid_map_msgs::GridMap& message);
    void localTraversabilityMapCallback(const grid_map_msgs::GridMapConstPtr& message);
    void fuseMap();
    //update map fusion at a certain frequency.
    void fuseMapCallback(const ros::TimerEvent&);
    //update map fusion once when click button.
    bool fuseMap(std_srvs::SetBool::Request& request,
                 std_srvs::SetBool::Response& response);
    bool loadGlobalTraversabilityMap(se2_navigation_msgs::RequestNavigationMapSrv::Request& request,
                                     se2_navigation_msgs::RequestNavigationMapSrv::Response& response);
    void timerThread();
    void publishFusedTraversabilityMap();
    void publishInitialTraversabilityMap();
    void publishLocalTraversabilityMap();

private:

    ros::NodeHandle nodehandle_;
    ros::CallbackQueue callback_queue_;
    //! WHY must initial it as class member
    ros::Timer update_timer;
    ros::Subscriber initialTraversabilityMapSubscriber_;
    ros::Subscriber localTraversabilityMapSubscriber_;
    ros::Publisher fusedTraversabilityMapPublisher_;
    ros::Publisher initialTraversabilityMapPublisher_;
    ros::Publisher localTraversabilityMapPublisher_;
    ros::ServiceServer mapFusionService_;
    ros::ServiceServer loadGlobalMapService_;
    GridMap initialTraversabilityMap_;
    GridMap localTraversabilityMap_;
    GridMap fusedTraversabilityMap_;
    boost::thread timer_thread;
    boost::recursive_mutex mutex_;
};

}
