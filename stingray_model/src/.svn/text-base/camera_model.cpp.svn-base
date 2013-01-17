/*------------------------------------------------------------------------------
 * Title:       camera_model.cpp
 * Description: ROS node for running a model of the Camera.
 *----------------------------------------------------------------------------*/

#include "camera_model.h"

/*------------------------------------------------------------------------------
 * CameraModel()
 * Constructor.
 *----------------------------------------------------------------------------*/

CameraModel::CameraModel()
{
} // end CameraModel()


/*------------------------------------------------------------------------------
 * ~CameraModel()
 * Destructor.
 *----------------------------------------------------------------------------*/

CameraModel::~CameraModel()
{
} // end ~CameraModel()


/*---------------------------------------------------------------------
* ms3dmgCallback()
* Callback for when compass data is received.
* -------------------------------------------------------------------*/

void CameraModel::ms3dmgCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    orientation_x = msg->orientation.x;
    orientation_y = msg->orientation.y;
    orientation_z = msg->orientation.z;
    orientation_w = msg->orientation.w;

    ROS_DEBUG("Compass Quaternions: %.1f\t%.1f\t%.1f\t%.1f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
} // end ms3dmgCallback()


/*---------------------------------------------------------------------
* os5000Callback()
* Callback for when compass data is received.
* -------------------------------------------------------------------*/

void CameraModel::os5000Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    orientation_x = msg->orientation.x;
    orientation_y = msg->orientation.y;
    orientation_z = msg->orientation.z;
    orientation_w = msg->orientation.w;

    ROS_DEBUG("Compass Quaternions: %.1f\t%.1f\t%.1f\t%.1f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
} // end os5000Callback()


/*---------------------------------------------------------------------
* main()
* Main function for ROS node.
* -------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "camera_model");
    ros::NodeHandle n;

    // Create a CameraModel object.
    CameraModel *camera_model = new CameraModel();

    // Declare variables.
    int mode = 1;
    std::string pub_gazebo_name;
    std::string sub_ms3dmg_topic;
    std::string sub_os5000_topic;
    int rate = 40;

    // Robot state variables.
    double angle = 0;

    // Message declarations.
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    tf::TransformBroadcaster broadcaster;

    // Initialize node parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("rate",              rate,                        int(20));
    private_node_handle_.param("mode",              mode,                        int(1));
    private_node_handle_.param("path_depth",        camera_model->path_depth ,   double(1.0));
    private_node_handle_.param("path_radius",       camera_model->path_radius,   double(1.0));
    private_node_handle_.param("sub_ms3dmg_topic",  sub_ms3dmg_topic,            std::string("mstrain_data"));
    private_node_handle_.param("sub_os5000_topic",  sub_os5000_topic,            std::string("os5000_data"));

    // Set up ROS subscribers and clients so that data can be found.
    ros::Subscriber sub_ms3dmg = n.subscribe(sub_ms3dmg_topic.c_str(), 1, &CameraModel::ms3dmgCallback, camera_model);
    ros::Subscriber sub_os5000 = n.subscribe(sub_os5000_topic.c_str(), 1, &CameraModel::os5000Callback, camera_model);

    // Tell ROS how fast to run this node.
    ros::Rate loop_rate(rate);

    // Main loop.
    while (ros::ok())
    {
        // Update transform.
        // (Moving in a circle with radius = path_radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0.0; //cos(angle) * camera_model->path_radius;
        odom_trans.transform.translation.y = 0.0; //sin(angle) * camera_model->path_radius;
        odom_trans.transform.translation.z = 0.0; //camera_model->path_depth;
        odom_trans.transform.rotation.x = camera_model->orientation_x;
        odom_trans.transform.rotation.y = camera_model->orientation_y;
        odom_trans.transform.rotation.z = camera_model->orientation_z;
        odom_trans.transform.rotation.w = camera_model->orientation_w;
        if (mode == 1)
        {
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle + M_PI / 2);
        }

        // Send the odometry transform.
        broadcaster.sendTransform(odom_trans);

        // Update the angle variable.
        //if (rate > 0)
        //{
        //    angle += M_PI / 180. / (rate / 10);
        //}
        //else
        //{
        //    angle = 0.0;
        //}

        // Let ROS sleep to keep proper loop rate.
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} // end main()
