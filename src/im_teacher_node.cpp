/*
 * @brief: インタラクティブマーカを発現及び操作するコード
 * @author: nosaka
*/

#include <ros/ros.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    double roll, pitch, yaw;

    // position
    ROS_INFO_STREAM(feedback->marker_name << "'s position is now at "
        << feedback->pose.position.x << ", "
        << feedback->pose.position.y << ", "
        << feedback->pose.position.z);

    // pose
    tf::Quaternion quat(feedback->pose.orientation.x,
        feedback->pose.orientation.y,
        feedback->pose.orientation.z,
        feedback->pose.orientation.w);

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM(feedback->marker_name << "'s rpy is now at "
        << roll << ", "
        << pitch << ", "
        << yaw << "\n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_marker");
    
    // create an interactive marker server on the topic namespace simple_marker
    interactive_markers::InteractiveMarkerServer server("simple_marker");

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;

    int_marker.header.frame_id = "N35_mounting_frame"; // "world"
    int_marker.name = "pose_teacher";
    int_marker.description = "pose_teacher";
    int_marker.scale = 0.2;

    // create a grey box marker
    visualization_msgs::Marker box_marker;

    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.02; // 0.45
    box_marker.scale.y = 0.02; // 0.45
    box_marker.scale.z = 0.02; // 0.45

    box_marker.color.r = 0.5; // 0.5
    box_marker.color.g = 0.5; // 0.5
    box_marker.color.b = 0.5; // 0.5
    box_marker.color.a = 1.0; // 1.0

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;

    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

    visualization_msgs::InteractiveMarkerControl control;

    tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, &processFeedback);

    // 'commit' changes and send to all clients
    server.applyChanges();

    // start the ROS main loop
    ros::spin();
}
