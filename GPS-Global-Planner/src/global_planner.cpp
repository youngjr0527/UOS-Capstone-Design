#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::SchoolGlobalPlanner, nav_core::BaseGlobalPlanner)




//Default Constructor
namespace global_planner {

SchoolGlobalPlanner::SchoolGlobalPlanner () : initialized_(false) {

}

SchoolGlobalPlanner::SchoolGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
: initialized_(false){
  initialize(name, costmap_ros);
}


void SchoolGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(initialized_){
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
      return;
  }
  ros::NodeHandle n("~/" + name);
  allpathpub = n.advertise<nav_msgs::Path>("all_paths",1,true);
  // __________________________
  // ROS PARAM
  std::string fpstring;
  n.param<string>("filepath",fpstring,"/home/robotics/catkin_ws/src/school_navigation/waypoints" );

  
  w.readWaypointFiles(fpstring);
  std::cout<<w.getNumEdges()<<std::endl;
  std::cout<<w.getNumNodes()<<std::endl;

  nav_msgs::Path all_p;
  all_p.header.frame_id="map";
  for(int i =0; i< w.getNumEdges(); i++){
    w.getpath(i,all_p);
  }
  allpathpub.publish(all_p);

  for(int i = 0;i<w.getNumNodes(); i++)
    d.create_Node();
  int n1,n2;
  float c1,c2;
  for(int i=0; i<w.getNumEdges(); i++){
    n1 = w.getEdgeNodeIdx1(i);
    n2 = w.getEdgeNodeIdx2(i);
    c1 = w.getEdgeCost1(i);
    c2 = w.getEdgeCost2(i);
    d.create_Edge(n1,c1,n2);
    d.create_Edge(n2,c2,n1);
  }

}

bool SchoolGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    int cEdgeIdx = 0;
    int cPoseIdx = 0;
    w.findClosest(start,cEdgeIdx,cPoseIdx);
    int added = w.addTmpWayp(cEdgeIdx,cPoseIdx, start);
    int numE = 0;
    int goalidx = 0;
    int n1,n2;
    float c1,c2;
    // add nodes, edges to dijkstra
    if(added== 1){
      goalidx = d.create_Node();
      numE = w.getNumEdges()-1;
      n1 = w.getEdgeNodeIdx1(numE);
      n2 = w.getEdgeNodeIdx2(numE);
      c1 = w.getEdgeCost1(numE);
      c2 = w.getEdgeCost2(numE);
      d.create_Edge(n1,c1,n2);
      d.create_Edge(n2,c2,n1);
    }
    else{
      goalidx = d.create_Node();
      // first edge
      numE = w.getNumEdges()-2;
      n1 = w.getEdgeNodeIdx1(numE);
      n2 = w.getEdgeNodeIdx2(numE);
      c1 = w.getEdgeCost1(numE);
      c2 = w.getEdgeCost2(numE);
      d.create_Edge(n1,c1,n2);
      d.create_Edge(n2,c2,n1);

      // second edge
      numE = w.getNumEdges()-1;
      n1 = w.getEdgeNodeIdx1(numE);
      n2 = w.getEdgeNodeIdx2(numE);
      c1 = w.getEdgeCost1(numE);
      c2 = w.getEdgeCost2(numE);
      d.create_Edge(n1,c1,n2);
      d.create_Edge(n2,c2,n1);
    }
    // d.print_nodes();


    nav_msgs::Path p;
    p.header.frame_id="map";

    int target = goal.pose.position.x;
    std::string n1Name;
    if(target == 1)
      n1Name = "A1";
    else if(target == 2)
      n1Name = "X1";
    else if(target == 3)
      n1Name = "X2";
    else if(target == 4)
      n1Name = "X4";
    else if(target == 5)
      n1Name = "X3";
    else
      n1Name = "A6";
    ROS_INFO("TARGET %d %s ",target, n1Name);
    
    n1 = w.getNodeIdx(n1Name);


    d.DijkstrasAlgorithm(goalidx);

    d.make_path(n1);
    d.print_path();
    // d.print_nodes();


    std::vector<int> path = d.get_path();


    for(int j = 1; j<path.size(); j++){
      int s1 = path.at(j-1);
      int s2 = path.at(j);
      w.getpath(s1,s2,p);
    }

    plan = p.poses;
    


    d.delete_Node();
    w.delTmpWayp(added);
    // d.print_nodes();
    return true;
  }  


};