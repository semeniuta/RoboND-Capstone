// Based on
// http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

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


int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle this_node;
    ros::Rate r(1. / 5.);

    double pickup_x, pickup_y, dropoff_x, dropoff_y;
    this_node.getParam("/home_service_robot/pickup_x", pickup_x);
    this_node.getParam("/home_service_robot/pickup_y", pickup_y);
    this_node.getParam("/home_service_robot/dropoff_x", dropoff_x);
    this_node.getParam("/home_service_robot/dropoff_y", dropoff_y);

    Mover mover;    

    mover.move_robot(pickup_x, pickup_y);
    r.sleep();

    mover.move_robot(dropoff_x, dropoff_y);
    r.sleep();

    return 0;
}