// Based on
// http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

const uint32_t MARKER_SHAPE = visualization_msgs::Marker::CUBE;
const double MARKER_SIZE = 0.2;

visualization_msgs::Marker prepare_marker(double x, double y) {
    
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type. 
    marker.type = MARKER_SHAPE;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = MARKER_SIZE / 2.;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = MARKER_SIZE;
    marker.scale.y = MARKER_SIZE;
    marker.scale.z = MARKER_SIZE;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    visualization_msgs::Marker marker = prepare_marker(0, 1);
    marker.action = visualization_msgs::Marker::ADD;

    while (ros::ok())
    {
        
        switch (marker.action) {
            
            case visualization_msgs::Marker::ADD:
                marker.action = visualization_msgs::Marker::DELETE;
                break;

            case visualization_msgs::Marker::DELETE:
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