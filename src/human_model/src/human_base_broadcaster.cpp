#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

class HelperClass
{
    public:
        HelperClass() = default;

        geometry_msgs::Quaternion Rot2Quaternion(double rotx, double roty, double rotz)
        // function that converts rotation around axis in Quaternions
        // Rot x,y,z (x red, y green, z blue)
        // Initial orientation (rotx=0,roty=0,rotz=0) the axis are here the one of the world coordinate system
        // When imagine rotations start always from this initial orientation and imagine a rotation of
        // the axis of the worl coordinate system
        // rotation clockwise (pos. rads)
        // double rotx = 0;
        // Rotation clockwise (pos. rads)
        // double roty = 0;
        // Rotation counterclockwise (pos. rads)
        // double rotz = 0;
        {
            double cx = cos(rotx * 0.5);
            double sx = sin(rotx * 0.5);
            double cy = cos(roty * 0.5);
            double sy = sin(roty * 0.5);
            double cz = cos(rotz * 0.5);
            double sz = sin(rotz * 0.5);

            geometry_msgs::Quaternion orientation;
            orientation.w = cx * cy * cz + sx * sy * sz;
            orientation.x = sx * cy * cz - cx * sy * sz;
            orientation.y = cx * sy * cz + sx * cy * sz;
            orientation.z = cx * cy * sz - sx * sy * cz;

            return orientation;
        }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "human_base");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster broadcaster;

    // Define a frame which can be referenced as human base relative to the world frame
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "human_base";

    // Define translation (in meters)
    transformStamped.transform.translation.x = 1.5;  // Adjust the x-coordinate
    transformStamped.transform.translation.y = 0.0;  // Adjust the y-coordinate
    transformStamped.transform.translation.z = 0.0;  // Adjust the z-coordinate

    HelperClass Helper;
    double rotx = M_PI/2;
    double roty = 0;
    double rotz = -M_PI/2;
    transformStamped.transform.rotation = Helper.Rot2Quaternion(rotx, roty, rotz);

    /*
    // Define rotation (as a quaternion)
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = -1.0;
    */

    // Set the timestamp
    transformStamped.header.stamp = ros::Time::now();

    // Broadcast the transform
    //TODO ADD a loop
    ros::Rate rate(100.0); // rate in Hz in which the transform should be send

    while (ros::ok())
    {
        transformStamped.header.stamp = ros::Time::now();        
        broadcaster.sendTransform(transformStamped);
        rate.sleep();        
    }

    return 0;
}
