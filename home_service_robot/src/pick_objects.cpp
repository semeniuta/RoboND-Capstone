// Based on
// http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal create_move_base_msg(double x, double y, double z, double yaw) {

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal.target_pose.pose.orientation.x = q[0];
    goal.target_pose.pose.orientation.y = q[1];
    goal.target_pose.pose.orientation.z = q[2];
    goal.target_pose.pose.orientation.w = q[3];

    return goal;
}

void log_send_goal(const move_base_msgs::MoveBaseGoal& goal) {

    ROS_INFO(
        "Sending goal: pos=[%.3f %.3f %.3f] orient=[%.3f %.3f %.3f %.3f]",
        goal.target_pose.pose.position.x,
        goal.target_pose.pose.position.y,
        goal.target_pose.pose.position.z,
        goal.target_pose.pose.orientation.x,
        goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z,
        goal.target_pose.pose.orientation.w
    );
}

class Mover {

private:
    //tell the action client that we want to spin a thread by default
    MoveBaseClient action_client_{"move_base", true};

public:

    Mover() {
        
        //wait for the action server to come up
        while (!action_client_.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    }


    void move_robot(double x, double y, double yaw = 0.) {

        move_base_msgs::MoveBaseGoal goal = create_move_base_msg(x, y, 0., yaw);

        log_send_goal(goal);
        action_client_.sendGoal(goal);
        action_client_.waitForResult();

        if (action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Destination reached");
        else
            ROS_INFO("The base failed to move");
    }

};

std_msgs::String create_state_msg(const char* s) {

    std_msgs::String msg;
    msg.data = s;
    
    return msg;
}

using PathViaPoints = std::vector<std::array<double, 2>>;

PathViaPoints read_path(char *filename) {

    std::ifstream in{filename};
    if (!in) {
        ROS_ERROR("Failure to open path configuration file");
    }

    std::string line;
    double x;
    double y;
    PathViaPoints points;

    while (std::getline(in, line)) {
        std::stringstream line_in{line};
        line_in >> x >> y;
        points.push_back({x, y});
    }

    return points;

}

int main(int argc, char **argv) {

    if (argc == 1) {
        ROS_ERROR("Provide file name of the path configuration file");
        return -1;
    }

    char *conf = argv[1];
    ROS_INFO("Path configuration file: %s", conf);

    auto path = read_path(conf);

    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle this_node;
    ros::Rate rate(1. / 5.);
    ros::Publisher state_pub = this_node.advertise<std_msgs::String>("/home_service_robot/robot_state", 1);
    
    double pickup_x = path[0][0];
    double pickup_y = path[0][1];

    unsigned int last_idx = path.size() - 1;
    double dropoff_x = path[last_idx][0];
    double dropoff_y = path[last_idx][1];

    Mover mover;

    state_pub.publish(create_state_msg("moving_to_pickup"));

    mover.move_robot(pickup_x, pickup_y);
    state_pub.publish(create_state_msg("at_pickup"));
    rate.sleep();

    state_pub.publish(create_state_msg("moving_to_dropoff"));

    double via_x, via_y;
    for (unsigned int i = 1; i < last_idx; i++) {
        via_x = path[i][0];
        via_y = path[i][1];
        ROS_INFO("Moving to via-point [%.2f, %.2f]", via_x, via_y);

        mover.move_robot(via_x, via_y);
    }

    mover.move_robot(dropoff_x, dropoff_y);
    state_pub.publish(create_state_msg("at_dropoff"));

    return 0;
}