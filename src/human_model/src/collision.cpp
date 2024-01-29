#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
// #include <tf2_ros/transform_listener.h> // useful to calculate the distance betweem two frames
#include <tf2_ros/transform_broadcaster.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <ros/console.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_model_node");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.d 
  ros::AsyncSpinner spinner(1);
  spinner.start();


  std::vector<std::string> joint_names;

  
   
  // Load the ros_parameters joint names (joint names of the bvh file)
  node_handle.getParam("joint_names", joint_names);
  
  /* Print the joint names
  ROS_INFO("Joint Names:");
  for (const auto& joint : joint_names) 
  {
    ROS_INFO("%s", joint.c_str());
  }
  */
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link8";
  collision_object.id = "Sphere1";

  // Define the sphere primitive
  shape_msgs::SolidPrimitive sphere_primitive;
  sphere_primitive.type = shape_msgs::SolidPrimitive::SPHERE;
  sphere_primitive.dimensions.resize(1);
  sphere_primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.1;

  /*
  // Define the pose of the spheres to avoid Warnings from rviz (doesnt help)
  geometry_msgs::Pose sphere_pose;
  sphere_pose.position.x = 0;
  sphere_pose.position.y = 0;
  sphere_pose.position.z = 0;
  sphere_pose.orientation.x = 0;
  sphere_pose.orientation.y = 0;
  sphere_pose.orientation.z = 0;
  sphere_pose.orientation.w = 0;
  */

  std::vector<moveit_msgs::CollisionObject> collision_objects;

    for (const auto& joint : joint_names) 
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = joint;  // Replace with your desired frame_id
        collision_object.id = joint;

        
        // Set the primitive for the collision object
        collision_object.primitives.push_back(sphere_primitive);

        // Set the pose of the collision object, assuming some default pose for simplicity
        collision_object.primitive_poses.push_back(geometry_msgs::Pose());

        // Add the collision object to the vector
        collision_objects.push_back(collision_object);
    }

   
  ros::Rate rate(100.0);  // Defining the refreshing rate of the while loop in Hz
  while (node_handle.ok())
  {
    for (std::size_t ii=0; ii < collision_objects.size(); ii++)
    {
      collision_object.operation = collision_objects[ii].ADD;
      planning_scene_interface.applyCollisionObject(collision_objects[ii]);
    }
    rate.sleep();
    //ROS_INFO("End of iteration");
  }


  ros::shutdown();
  return 0;
}
