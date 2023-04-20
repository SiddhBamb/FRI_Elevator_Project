#include <queue>
#include <inttypes.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* action_client;

std::queue<uint64_t> previous_classifications;
uint64_t num_open_classifications;
std::queue<uint64_t> previous_inside_predictions;
uint64_t num_open_from_inside;

const std::string door_open_msg = "Door open";
const std::string door_closed_msg = "Door closed";

bool inElevator;
bool currentlyMoving;
bool facingDoorFromInside;


void rotate180Degrees()
{

    if (currentlyMoving) return;

    currentlyMoving = true;

    move_base_msgs::MoveBaseGoal rotationTarget;

    rotationTarget.target_pose.pose.orientation.x = 0;
    rotationTarget.target_pose.pose.orientation.y = 0;
    rotationTarget.target_pose.pose.orientation.z = -1.0;
    rotationTarget.target_pose.pose.orientation.w = 1.0;

    action_client->sendGoal(rotationTarget);
    action_client->waitForResult();

    currentlyMoving = false;

    if(action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        facingDoorFromInside = true;
        ROS_INFO("The robot rotated 180 degrees successfully");

    } else {

        ROS_INFO("The robot failed to rotate");

    }

}


void moveIntoElevator()
{

    if (currentlyMoving || inElevator) return;

    currentlyMoving = true;

    move_base_msgs::MoveBaseGoal elevatorTarget;

    elevatorTarget.target_pose.header.frame_id = "base_link";
    elevatorTarget.target_pose.header.stamp = ros::Time::now();

    elevatorTarget.target_pose.pose.position.x = 2.0;
    elevatorTarget.target_pose.pose.orientation.w = 1.0;

    action_client->sendGoal(elevatorTarget);
    action_client->waitForResult();

    currentlyMoving = false;

    if(action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        inElevator = true;
        while (!previous_classifications.empty()) previous_classifications.pop();
        ROS_INFO("The robot should now be in the elevator");

        rotate180Degrees();

    } else {

        ROS_INFO("The robot failed to move into the elevator");

    }

}


void elevatorDoorClassificationCallback(const std_msgs::String::ConstPtr& msg)
{

    if (currentlyMoving || inElevator) return;

    std::string curr_msg = msg->data.c_str();
    if (curr_msg.compare(door_open_msg) == 0) {
        previous_classifications.push(1);
        num_open_classifications++;
        ROS_INFO("The current classification is door open");
    } else if (curr_msg.compare(door_closed_msg) == 0) {
        previous_classifications.push(0);
        ROS_INFO("The current classification is door closed");
    }

    if (previous_classifications.size() > 5) {
        uint64_t oldest_status = previous_classifications.front();
        previous_classifications.pop();
        num_open_classifications -= oldest_status;
        if (num_open_classifications >= 4) {
            moveIntoElevator();
        }
    }

}


void moveOutOfElevator()
{

    if (currentlyMoving || !inElevator || !facingDoorFromInside) return;

    currentlyMoving = true;

    move_base_msgs::MoveBaseGoal outsideTarget;

    outsideTarget.target_pose.header.frame_id = "base_link";
    outsideTarget.target_pose.header.stamp = ros::Time::now();

    outsideTarget.target_pose.pose.position.x = 2.0;
    outsideTarget.target_pose.pose.orientation.w = 1.0;

    action_client->sendGoal(outsideTarget);
    action_client->waitForResult();

    currentlyMoving = false;

    if(action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        inElevator = false;
        facingDoorFromInside = false;
        while (!previous_inside_predictions.empty()) previous_inside_predictions.pop();
        ROS_INFO("The robot should now be outside the elevator");

    } else {

        ROS_INFO("The robot failed to move outside the elevator");

    }
}


void elevatorDoorOpenFromInsideCallback(const std_msgs::String::ConstPtr& msg)
{

    if (currentlyMoving || !inElevator || !facingDoorFromInside) return;

    std::string curr_msg = msg->data.c_str();
    if (curr_msg.compare(door_open_msg) == 0) {
        previous_inside_predictions.push(1);
        num_open_from_inside++;
        ROS_INFO("The current prediction is door open from inside");
    } else if (curr_msg.compare(door_closed_msg) == 0) {
        previous_inside_predictions.push(0);
        ROS_INFO("The current prediction is door closed from inside");
    }

    if (previous_inside_predictions.size() > 5) {
        uint64_t oldest_status = previous_inside_predictions.front();
        previous_inside_predictions.pop();
        num_open_from_inside -= oldest_status;
        if (num_open_from_inside >= 4) {
            moveOutOfElevator();
        }
    }

}

int main(int argc, char **argv)
{

    num_open_classifications = 0;
    num_open_from_inside = 0;
    inElevator = false;
    currentlyMoving = false;
    facingDoorFromInside = false;

    ros::init(argc, argv, "elevator_movement");
    ros::NodeHandle n;

    ros::Subscriber classificationSubscriber = n.subscribe("elevator_door_classification", 1, elevatorDoorClassificationCallback);
    ros::Subscriber elevatorDoorOpenFromInsideSubscriber = n.subscribe("elevator_door_open_from_inside", 1, elevatorDoorOpenFromInsideCallback);

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action client");
    }
    action_client = &ac;

    ros::spin();

    return 0;

}