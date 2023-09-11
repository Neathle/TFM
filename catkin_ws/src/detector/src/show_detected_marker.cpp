#include "ros/ros.h"
#include <stdio.h>
#include <detector/messagedet.h>
#include <detector/marker.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



class Detected{


    public: ros::NodeHandle n;
    private: ros::NodeHandle private_nh_;
    private: void showMarkerCallback(const detector::messagedet::ConstPtr &msg);
    public: Detected();
    private: ros::Subscriber sub_marker;
    private: XmlRpc::XmlRpcValue marker_list;
    private: std::vector<int> IDs,sectors,maps,x,y,z;
    private: std::vector<visualization_msgs::MarkerArray> show_marker;

};


int main(int argc, char **argv){
    ros::init(argc, argv, "show_detected_marker");
    Detected ic;
    ros::spin();

    return 0;
}

Detected::Detected(){

    private_nh_.getParam("marker_positions", marker_list);
    ros::Subscriber sub = n.subscribe("num_detecte_markers", 1, &Detected::showMarkerCallback, this);

    for(int i=0;i<marker_list.size();i++)
    {
        IDs.push_back(marker_list[i]["ID"]);
        sectors.push_back(marker_list[i]["sector"]);
        maps.push_back(marker_list[i]["map"]);
        x.push_back(marker_list[i]["x"]);
        y.push_back(marker_list[i]["y"]);
        z.push_back(marker_list[i]["z"]);
        visualization_msgs::Marker marker;
        marker.id = uint32_t(i);
        marker.type = 1;
        marker.action = 0;
        marker.pose.position.x = marker_list[i]["x"];
        marker.pose.position.y = marker_list[i]["y"];
        marker.pose.position.z = marker_list[i]["z"];
        marker.color.a = 1;
        marker.color.r = 1;

    }
}

void Detected::showMarkerCallback(const detector::messagedet::ConstPtr &msg)
{

}

