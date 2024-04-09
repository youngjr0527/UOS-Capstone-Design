#include <iostream>
#include <string>
#include <vector>
#include <queue> // To set up priority queue
#include <functional> // To use std::greater<T> 
#include <ros/ros.h>
#include <experimental/filesystem>
#include <fstream>
#include <sstream>
#include <nav_msgs/Path.h>
#include <algorithm>
// includes PoseStamped, Pose, Point, Quaternion
// #include <geometry_msgs/PoseStamped.h>  // -> this is already nav/in path.h
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef std::pair<int, int> Pair;
typedef std::pair<double, double> Pairf;
class Dijkstra{
  public:
  class Node{
    public:
    int idx=-1;
    int parent_idx=-1;
    float path_length=-1;
    bool visited=false;
    std::vector <Pair> adjList;

    bool operator<(const Node s) const{
      if (this->path_length < 0) // 0보다 작으면 무한대
        return false;
      else if (s.path_length < 0) // 0보다 작으면 무한대
        return true;        
      return this->path_length < s.path_length;
    }
  };
// 생성자  
  Dijkstra(int num_Nodes){
    this->num_Nodes = num_Nodes;
    
    for (int count = 0 ; count<num_Nodes; count++){
      node_list.push_back(Node());
      node_list[count].idx = count;
    }
  }

  Dijkstra(){}

  int create_Node(){
    int count = this->num_Nodes;
    node_list.push_back(Node());
    node_list[count].idx = count;
    this->num_Nodes++;
    return count;
  }

  // 이웃 edge들도 모두 삭제한다.
  int delete_Node(){
    int count = this->num_Nodes-1;
    for(auto nei = node_list[count].adjList.begin(); nei != node_list[count].adjList.end(); nei++) {
      node_list[nei->second].adjList.pop_back();
    }
    node_list.pop_back();
    this->num_Nodes--;
  }

  void create_Edge(int u, float weight, int v){
    node_list[u].adjList.push_back( std::make_pair(weight,v) );
  }



  void DijkstrasAlgorithm(int start){
    this->start = start;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair> > PQ; // Set up priority queue
    // std::priority_queue< Node > PQ; // Set up priority queue
    Node current;
    Node next;
    Pair current_info;

    for (std::vector<Node>::iterator it = node_list.begin(); it !=node_list.end(); it++){
      it->path_length=-1;
      it->visited = false;
      it->parent_idx = -1;
    }
    node_list[start].path_length = 0;
    
    PQ.push(std::make_pair(0, start)); // Source has weight 0;

    while( !PQ.empty() ){
      current_info = PQ.top();
      PQ.pop();

      current = node_list[current_info.second];

      if(current.visited){
        continue;
      }
      current.visited = true;
      node_list[current_info.second] = current;

      next = current;
      for(std::vector<Pair>::iterator it = current.adjList.begin(); it !=current.adjList.end(); it++){
        next.path_length = current.path_length + (it->first );
        // std::cout << next.path_length << " " << node_list[it->second].path_length << " " << (next < node_list[it->second])<<std::endl;
        if (next < node_list[it->second]){
          node_list[it->second].parent_idx = current.idx;
          node_list[it->second].path_length = next.path_length;
          PQ.push(std::make_pair(node_list[it->second].path_length, it->second)); // Push vertex and weight onto Priority Queue
        }
      }
    }
  }

  void make_path(int end){
    path.clear();
    if (end==start){
      path.push_back(end);
    }
    else if (node_list[end].parent_idx == -1){
      ROS_ERROR("[DIJKSTRA] wrong path");
    }
    else{
      make_path(node_list[end].parent_idx);
      path.push_back(end);
    }
  }

  std::vector<int> get_path(){
    return path;
  }

  void print_path(){
    std::cout<< "shortest path is : ";
    for (std::vector<int>::iterator it = path.begin(); it!=path.end(); it++){
      std::cout << *it << " ";
    }
    std::cout<<std::endl;
  }

  void print_nodes(){
    for (std::vector<Node>::iterator it = node_list.begin(); it !=node_list.end(); it++){
      std::cout <<"idx :" << it->idx <<" parent :"<< it-> parent_idx<<" path_len :"<< it->path_length << " visited :" << it->visited << " nei : ";
      for( auto nei = it->adjList.begin(); nei != it->adjList.end(); nei++){
        std::cout<< nei->second <<" ";
      }
      std::cout<<std::endl;
    }
  }

  // total number of nodes
  int num_Nodes=0;
  std::vector<Node> node_list;
  std::vector<int> path;
  int start;
};


class WaypointInfo{
  // fields
  std::vector<std::string> nodeList;
  std::vector<Pair> edgeList;
  std::vector<std::vector<geometry_msgs::PoseStamped>> edgeWayList;
  std::vector<Pairf> costList;

public:
  int getNumNodes(){
    return nodeList.size();
  }
  int getNumEdges(){
    return edgeList.size();
  }

  int getNodeIdx(std::string name){
    int numNodes = getNumNodes();
    for(int i =0; i<numNodes; i++ ){
      if(name.compare(nodeList.at(i)) ==0)
        return i;
    }
    return -1;
  }

  int addNodeIdx(std::string name){
    nodeList.push_back(name);
    return getNumNodes()-1;
  }
  

  int getEdgeIdx(int n1, int n2){
    int numEdges = getNumEdges();
    for(int i=0; i<numEdges; i++){
      Pair tmp = edgeList.at(i);
      if ((tmp.first == n1) && (tmp.second == n2))
        return i;
      else if ((tmp.first == n2) && (tmp.second == n1))
        return i;
    }
    return -1;
  }


  int addEdgeIdx(int n1, int n2){
    Pair edge(n1,n2);
    edgeList.push_back(edge);
    return getNumEdges()-1;
  }

  void delEdgeIdx(){
    edgeList.pop_back();
  }

  void addEdgeWayList(std::vector<geometry_msgs::PoseStamped> plist){
    edgeWayList.push_back(plist);
  }

  void delEdgeWayList(){
    edgeWayList.pop_back();
  }


  void addCostList(Pairf cost){
    costList.push_back(cost);
  }
  void delCostList(){
    costList.pop_back();
  }

  int getEdgeNodeIdx1(int edgeIdx){
    return edgeList.at(edgeIdx).first;
  }

  int getEdgeNodeIdx2(int edgeIdx){
    return edgeList.at(edgeIdx).second;
  }

  double getEdgeCost1(int edgeIdx){
    return costList.at(edgeIdx).first;
  }

  double getEdgeCost2(int edgeIdx){
    return costList.at(edgeIdx).second;
  }

  double inline calcSqDis(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return (dx*dx +dy*dy);
  }

public:
  bool getpath(int n1, int n2, nav_msgs::Path &path){
    int numEdges = getNumEdges();
    bool reverse = false;
    int i = 0;
    for(i=0; i<numEdges; i++){
      Pair tmp = edgeList.at(i);
      if ((tmp.first == n1) && (tmp.second == n2)){
        reverse = false;
        break;
      }
      else if ((tmp.first == n2) && (tmp.second == n1)){
        reverse = true;
        break;
      }
    }

    if(i== numEdges){
      ROS_ERROR("[getpath] NO such edges! %d %d",n1,n2);
      return false;
    }
    
    int numWay = edgeWayList.at(i).size();
    if(!reverse){
      for (int j = 0 ; j<numWay; j++)
        path.poses.push_back(edgeWayList[i][j]);
    }
    else if(reverse){
      for (int j = numWay-1 ; j>=0; j--)
        path.poses.push_back(edgeWayList[i][j]);
    }
    return true;
  }

  bool getpath(int edgeIdx, nav_msgs::Path &path){
    int numWay = edgeWayList.at(edgeIdx).size();
    for (int j = 0 ; j<numWay; j++)
      path.poses.push_back(edgeWayList[edgeIdx][j]);

    return true;
  }

  bool readWaypointFiles(std::string fp){
    std::experimental::filesystem::path filepath(fp);
    if (std::experimental::filesystem::exists(filepath)){
      std::experimental::filesystem::directory_iterator fileitr(filepath);
      while (fileitr != std::experimental::filesystem::end(fileitr)) {
        const std::experimental::filesystem::directory_entry& entry = *fileitr;
        std::cout << entry.path() << std::endl;
        

        // read file
        std::ifstream input_file(entry.path());
        if (!input_file.is_open()) {
          ROS_ERROR( "Could not open the file - %s",entry.path() );
          return false;
        }

        std::string name;
        std::getline(input_file, name);

        // get edge name
        std::string n1Name = name.substr(1,2);
        std::string n2Name = name.substr(3,2);
        int n1 = getNodeIdx(n1Name);
        if (n1 == -1)
          n1 = addNodeIdx(n1Name);

        int n2 = getNodeIdx(n2Name);
        if (n2 == -1)
          n2 = addNodeIdx(n2Name);

        int edge = getEdgeIdx(n1, n2);
        if (edge == -1){
          edge = addEdgeIdx(n1, n2);
        }
        else {
          ROS_ERROR("Same Waypoint information deteced!- %s",entry.path() );
          input_file.close();
          return false;
        }

        std::vector<geometry_msgs::PoseStamped> plist;

        double d1;
        double d2;
        bool first=true;
        while(!input_file.eof()){
          input_file >> d1;
          input_file >> d2;
          if (first){
            Pairf cost(d1,d2);
            addCostList(cost);
            first = false;
            continue;
          }
          // ROS_INFO(" %12.6f %12.6f ",d1,d2);
          geometry_msgs::PoseStamped p;
          p.header.frame_id="map";
          p.pose.position.x = d1-328540.00;
          p.pose.position.y = d2-4161354.00;
          p.pose.orientation.w = 1;

          plist.push_back(p);
        }
        addEdgeWayList(plist);
        // std::cout<<"HERE"<<plist.at(0).pose.position.x<<std::endl;
        input_file.close();
        fileitr++;
      }
    }
    else{
      ROS_ERROR("NO such directory!");
      return false;
    }
    return true;
  }

  void findClosest(const geometry_msgs::PoseStamped& start, int& cEdgeIdx, int& cPoseIdx ){
    int minEdgeIdx = 0;
    int minPoseIdx = 0;
    double minSqDis = 100000.0;
    double tmpSqDis = 0.0;
    for(int edge = 0; edge < edgeWayList.size(); edge++){
      for(int pose=0; pose < edgeWayList[edge].size(); pose++){
        tmpSqDis = calcSqDis(edgeWayList[edge][pose],start);
        if(minSqDis > tmpSqDis){
          minSqDis = tmpSqDis;
          minEdgeIdx = edge;
          minPoseIdx = pose;
        }
      }
    }
    cEdgeIdx = minEdgeIdx;
    cPoseIdx = minPoseIdx;
  }

  std::vector<geometry_msgs::PoseStamped> makeLinearInter(const geometry_msgs::PoseStamped& p1,const geometry_msgs::PoseStamped& p2){
    double dx = p2.pose.position.x - p1.pose.position.x;
    double dy = p2.pose.position.y - p1.pose.position.y;
    double dist = sqrt(calcSqDis(p1,p2));
    double stepx  =  dx / dist * 0.1;
    double stepy  =  dy / dist * 0.1;
    std::vector<geometry_msgs::PoseStamped> newPath;

    double px = p1.pose.position.x;
    double py = p1.pose.position.y;


    double pp = 0.0;
    while(pp + 0.1 < dist){
      geometry_msgs::PoseStamped p;
      p.header.frame_id="map";
      p.pose.position.x = px;
      p.pose.position.y = py;
      p.pose.orientation.w = 1;
      newPath.push_back(p);
      pp = pp + 0.1;
      px = px + stepx;
      py = py + stepy;
    }

    return newPath;
  }

  int addTmpWayp(int edge, int pose, const geometry_msgs::PoseStamped& start){
    std::vector<geometry_msgs::PoseStamped> tmp = makeLinearInter(edgeWayList[edge][pose], start);
    std::vector<geometry_msgs::PoseStamped> ori = edgeWayList[edge];
    int n1,n2,n3;
    n1 = getEdgeNodeIdx1(edge);
    n2 = getEdgeNodeIdx2(edge);
    n3 = addNodeIdx("tmp");
    if(pose==0 || pose == edgeWayList[edge].size()-1){
      auto it = ori.begin();
      std::advance(it, pose);
      std::vector<geometry_msgs::PoseStamped>tmp2(ori.begin(),it);
      tmp2.insert(tmp2.end(), tmp.begin(), tmp.end());

      addEdgeIdx(n1,n3);
      addEdgeWayList(tmp2);
      addCostList(Pairf(getEdgeCost1(edge),getEdgeCost2(edge)));
      return 1;
    }
    else{
      auto it = ori.begin();
      std::advance(it, pose);
      std::vector<geometry_msgs::PoseStamped>tmp2(ori.begin(),it);
      tmp2.insert(tmp2.end(), tmp.begin(), tmp.end());


      auto it2 = ori.begin();
      std::advance(it2, pose);
      std::reverse(tmp.begin(), tmp.end());
      tmp.insert(tmp.end(), it2 , ori.end());


      addEdgeIdx(n1,n3);
      addEdgeIdx(n3,n2);
      addEdgeWayList(tmp2);
      addEdgeWayList(tmp);
      addCostList(Pairf(getEdgeCost1(edge)/2,getEdgeCost2(edge)/2));
      addCostList(Pairf(getEdgeCost1(edge)/2,getEdgeCost2(edge)/2));

      return 2;
    }
  }

  void delTmpWayp(int added){
    nodeList.pop_back();
    if(added==1){
      delEdgeIdx();
      delEdgeWayList();
      delCostList();
    }
    else{
      delEdgeIdx();
      delEdgeIdx();
      delEdgeWayList();
      delEdgeWayList();
      delCostList();
      delCostList();

    }
  }
};
