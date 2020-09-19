#include <se2_planning/UpdateMapFusion.hpp>

using namespace grid_map;

namespace se2_planning{

TraversabilityMapFusion::TraversabilityMapFusion(ros::NodeHandle& nh)
    :nodehandle_(nh)
{
    localTraversabilityMapSubscriber_ = nodehandle_.subscribe(
                "/traversability_estimation_rt_node/traversability_map", 1, &TraversabilityMapFusion::localTraversabilityMapCallback, this);
//    localTraversabilityMapSubscriber_ = nodehandle_.subscribe(
//                "/grid_map_simple_demo/grid_map", 100, &TraversabilityMapFusion::localTraversabilityMapCallback, this);
    fusedTraversabilityMapPublisher_ = nodehandle_.advertise<grid_map_msgs::GridMap>("fused_traversability_map", 100);
    initialTraversabilityMapPublisher_ = nodehandle_.advertise<grid_map_msgs::GridMap>("initial_traversability_map", 100);
    //localTraversabilityMapPublisher_ = nodehandle_.advertise<grid_map_msgs::GridMap>("local_traversability_map", 100);
    mapFusionService_ = nodehandle_.advertiseService("map_fusion", &TraversabilityMapFusion::fuseMap, this);
    loadGlobalMapService_ = nodehandle_.advertiseService("load_global_map", &TraversabilityMapFusion::loadGlobalTraversabilityMap, this);
    // Wait subscribe map info.
    //sleep(1);
    // Update map fusion at a certain frequency.
//    ros::TimerOptions control_time_option(ros::Duration(1), // 1 Hz
//                                          boost::bind(&TraversabilityMapFusion::fuseMapCallback, this, _1),
//                                          &callback_queue_, false, false);
//    update_timer = nodehandle_.createTimer(control_time_option);
//    timer_thread = boost::thread(boost::bind(&TraversabilityMapFusion::timerThread, this));
//    update_timer.start();
    ROS_INFO("Traversability map fusion class constructed.");
}

TraversabilityMapFusion::~TraversabilityMapFusion()
{
    ROS_INFO("Descructed map fusion node. ");
}

bool TraversabilityMapFusion::loadInitialTraversabilityMap()
{
    ROS_WARN("Loading initial traversability map");
    boost::recursive_mutex::scoped_lock scopedLock(mutex_);
    std::string topic =
            "/traversability_estimation/traversability_map";
    std::string path_file =
            "/home/eric/traversability_map.bag";
    GridMap gridMap;
    GridMapRosConverter::loadFromBag(path_file, topic, gridMap);
    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));
    initialTraversabilityMap_ = gridMap;
    return true;
}

bool TraversabilityMapFusion::loadLocalTraversabilityMap()
{
    ROS_WARN("Loading local traversability map");
    boost::recursive_mutex::scoped_lock scopedLock(mutex_);
    std::string topic =
            "/traversability_estimation/traversability_map";
    std::string path_file =
            "/home/eric/local_traversability_map.bag";
    GridMap gridMap;
    GridMapRosConverter::loadFromBag(path_file, topic, gridMap);
    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));
    localTraversabilityMap_ = gridMap;
    return true;
}

void TraversabilityMapFusion::initialTraversabilityMapCallback(const grid_map_msgs::GridMap &message)
{
    ROS_INFO_ONCE("Receive local traversability map");
    GridMap gridMap;
    GridMapRosConverter::fromMessage(message, gridMap);
    ROS_INFO_ONCE("Created initial map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));
    initialTraversabilityMap_ = gridMap;
}

void TraversabilityMapFusion::localTraversabilityMapCallback(const grid_map_msgs::GridMapConstPtr &message)
{
    ROS_INFO_ONCE("Receive local traversability map.");
    GridMap gridMap;
    GridMapRosConverter::fromMessage(*message, gridMap);
//    if(!gridMap.exists("traversability"))
//        gridMap.add("traversability", 1);
    ROS_INFO_ONCE("Created local map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));
    localTraversabilityMap_ = gridMap;
}

bool TraversabilityMapFusion::loadGlobalTraversabilityMap(se2_navigation_msgs::RequestNavigationMapSrv::Request &request,
                                                    se2_navigation_msgs::RequestNavigationMapSrv::Response &response)
{
    ROS_INFO("Start loading global traversability map.");
    std::string topic = request.topic_name;
    std::string path_file = request.file_path;
    GridMap gridMap;
    grid_map_msgs::GridMap message_;
    GridMapRosConverter::loadFromBag(path_file, topic, gridMap);
    initialTraversabilityMap_ = gridMap;
    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));
    GridMapRosConverter::toMessage(gridMap, message_);
    initialTraversabilityMapPublisher_.publish(message_);
    fusedTraversabilityMap_ = gridMap;
    //Publish initial map as fused map once when load initial map.
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(fusedTraversabilityMap_, message);
    fusedTraversabilityMapPublisher_.publish(message);
    ROS_WARN("Load global traversability map success.");
    return true;
}

void TraversabilityMapFusion::fuseMap()
{
    //* Use local map to update global map
    while(ros::ok())
    {
        GridMap gridMapGlobal;
        gridMapGlobal = initialTraversabilityMap_;
        GridMap gridMapLocal;
        gridMapLocal = localTraversabilityMap_;
        grid_map::Matrix& data = gridMapGlobal["traversability"];
        grid_map::Matrix& dataToAdd = gridMapLocal["traversability"];
        for(grid_map::GridMapIterator iterator(gridMapLocal); !iterator.isPastEnd(); ++iterator)
        {
            const Index index(*iterator);
            //ROS_WARN("Starting fusing map");
            //auto& traversability = gridMapGlobal.at("traversability", index);
            //auto& elevation = gridMapGlobal.at("elevation", index);
            //ROS_WARN("Get traversability layer of the fused map");
            if(!gridMapLocal.isValid(index, "traversability"))
                continue; // ignore cells without data in local map
            //ROS_WARN_STREAM_ONCE("Started Index: " << std::endl << index << std::endl);
            //traversability = gridMapLocal.at("traversability", index);
            data(index(0), index(1)) = dataToAdd(index(0), index(1));
            //elevation = gridMapLocal.at("elevation", index);
            //ROS_WARN_ONCE("Get traversability layer of the initial map");
        }
        // fuse finish
        fusedTraversabilityMap_ = gridMapGlobal;
        publishFusedTraversabilityMap();
        publishInitialTraversabilityMap();
        publishLocalTraversabilityMap();
    }
}

void TraversabilityMapFusion::fuseMapCallback(const ros::TimerEvent&)
{
    ROS_WARN("spinOnce");
    ros::Time start_time = ros::Time::now();
    boost::recursive_mutex::scoped_lock lock(mutex_);
    GridMap gridMapGlobal;
    gridMapGlobal = initialTraversabilityMap_;
    GridMap gridMapLocal;
    gridMapLocal = localTraversabilityMap_;
    grid_map::Matrix& data1= gridMapGlobal["elevation"];
    grid_map::Matrix& dataToAdd1 = gridMapLocal["elevation"];
    grid_map::Matrix& data2 = gridMapGlobal["traversability"];
    grid_map::Matrix& dataToAdd2 = gridMapLocal["traversability"];
    for(grid_map::GridMapIterator iterator(gridMapLocal); !iterator.isPastEnd(); ++iterator)
    {
        const Index index(*iterator);
        if(!gridMapLocal.isValid(index, "elevation"))
            continue; // ignore cells without data in local map
        data1(index(0), index(1)) = dataToAdd1(index(0), index(1));
        data2(index(0), index(1)) = dataToAdd2(index(0), index(1));
    }
    // fuse finish
    fusedTraversabilityMap_ = gridMapGlobal;
    lock.unlock();
    publishFusedTraversabilityMap();
    publishInitialTraversabilityMap();
    publishLocalTraversabilityMap();
    ros::Time end_time = ros::Time::now();
    ros::Duration duration_ = (end_time - start_time);
    ROS_WARN_STREAM("Duration time for spin once: " << duration_ << " s;" << std::endl);
}

bool TraversabilityMapFusion::fuseMap(std_srvs::SetBool::Request &request,
                                      std_srvs::SetBool::Response &response)
{
    ROS_WARN("Fuse Map Once. ");
    ros::Time start_time = ros::Time::now();
    boost::recursive_mutex::scoped_lock lock(mutex_);
    GridMap gridMapGlobal;
    gridMapGlobal = initialTraversabilityMap_;
    GridMap gridMapLocal;
    gridMapLocal = localTraversabilityMap_;
    grid_map::Matrix& data1= gridMapGlobal["elevation"];
    grid_map::Matrix& dataToAdd1 = gridMapLocal["elevation"];
    grid_map::Matrix& data2 = gridMapGlobal["traversability"];
    grid_map::Matrix& dataToAdd2 = gridMapLocal["traversability"];
    for(grid_map::GridMapIterator iterator(gridMapLocal); !iterator.isPastEnd(); ++iterator)
    {
        const Index index(*iterator);
        if(!gridMapLocal.isValid(index, "elevation"))
            continue; // ignore cells without data in local map
        data1(index(0), index(1)) = dataToAdd1(index(0), index(1));
        data2(index(0), index(1)) = dataToAdd2(index(0), index(1));
    }
    // fuse finish
    fusedTraversabilityMap_ = gridMapGlobal;
    lock.unlock();
    // publish all map at the same time.
    publishFusedTraversabilityMap();
    publishInitialTraversabilityMap();
    publishLocalTraversabilityMap();
    ros::Time end_time = ros::Time::now();
    ros::Duration duration_ = (end_time - start_time);
    ROS_WARN_STREAM("Duration time for spin once: " << duration_ << " s;" << std::endl);
}

void TraversabilityMapFusion::timerThread()
{
    static const double timeout = 1;
    while(nodehandle_.ok())
    {
        callback_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void TraversabilityMapFusion::publishFusedTraversabilityMap()
{
    grid_map_msgs::GridMap message;
    GridMap gridMap;
    gridMap = fusedTraversabilityMap_;
    GridMapRosConverter::toMessage(gridMap, message);
    fusedTraversabilityMapPublisher_.publish(message);
}

void TraversabilityMapFusion::publishInitialTraversabilityMap()
{
    grid_map_msgs::GridMap message;
    GridMap gridMap;
    gridMap = initialTraversabilityMap_;
    GridMapRosConverter::toMessage(gridMap, message);
    initialTraversabilityMapPublisher_.publish(message);
}

void TraversabilityMapFusion::publishLocalTraversabilityMap()
{
    grid_map_msgs::GridMap message;
    GridMap gridMap;
    gridMap = localTraversabilityMap_;
    GridMapRosConverter::toMessage(gridMap, message);
    localTraversabilityMapPublisher_.publish(message);
}

}
