#include "markers.h"
#include <std_msgs/String.h>

const MarkerPosition PICKUP_POS{0, 1};
const MarkerPosition DROPOFF_POS{6, -2};

class MarkerManager {

private:

    ros::Publisher marker_pub_;

    void publish(const visualization_msgs::Marker& marker) {

        while (marker_pub_.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub_.publish(marker);

    }

public:

    explicit MarkerManager(ros::NodeHandle n) {
        
        marker_pub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 5);

        auto marker = prepare_marker(PICKUP_POS);
        marker.action = visualization_msgs::Marker::ADD;
        publish(marker);
    }

    void on_state_changed(const std_msgs::String& state_msg) {

        const char *s = state_msg.data.c_str();

        if (std::strcmp(s, "at_pickup") == 0) {

            auto marker = prepare_marker(PICKUP_POS);
            marker.action = visualization_msgs::Marker::DELETE;
            publish(marker);
            
        } else if (std::strcmp(s, "at_dropoff") == 0) {

            auto marker = prepare_marker(DROPOFF_POS);
            marker.action = visualization_msgs::Marker::ADD;
            publish(marker);
        }

     }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_markers");
    ros::NodeHandle this_node;

    MarkerManager manager{this_node};

    ros::Subscriber odom_sub = this_node.subscribe("/home_service_robot/robot_state", 5, &MarkerManager::on_state_changed, &manager);

    ros::spin();
}