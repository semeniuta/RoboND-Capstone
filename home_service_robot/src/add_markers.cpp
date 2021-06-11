// Based on
// http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

#include "markers.h"

const MarkerPosition PICKUP_POS{0, 1};
const MarkerPosition DROPOFF_POS{6, -2};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1. / 5.);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    Cycler cycler{PICKUP_POS, DROPOFF_POS};

    visualization_msgs::Marker marker = prepare_marker(cycler.getCurrentPosition());
    marker.action = visualization_msgs::Marker::ADD;
    
    while (ros::ok()) {
        
        switch (marker.action) {
            
            case visualization_msgs::Marker::ADD:
            
                marker.action = visualization_msgs::Marker::DELETE;
            
                break;

            case visualization_msgs::Marker::DELETE:

                cycler.next();
                marker = prepare_marker(cycler.getCurrentPosition());
                marker.action = visualization_msgs::Marker::ADD;
            
                break;
        } 

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);

        r.sleep();
    }
}