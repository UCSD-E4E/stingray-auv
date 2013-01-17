/*------------------------------------------------------------------------------
 * Title:       stingray_model.cpp
 * Description: ROS node for running a model of the Stingray.
 *----------------------------------------------------------------------------*/

#include "stingray_model.h"

/*------------------------------------------------------------------------------
 * StingrayModel()
 * Constructor.
 *----------------------------------------------------------------------------*/

StingrayModel::StingrayModel()
{
} // end StingrayModel()


/*------------------------------------------------------------------------------
 * ~StingrayModel()
 * Destructor.
 *----------------------------------------------------------------------------*/

StingrayModel::~StingrayModel()
{
} // end ~StingrayModel()


/*---------------------------------------------------------------------
* ms3dmgCallback()
* Callback for when compass data is received.
* -------------------------------------------------------------------*/

void StingrayModel::ms3dmgCallback(const sensor_msgs::Imu::ConstPtr &msg)
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

void StingrayModel::os5000Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    orientation_x = msg->orientation.x;
    orientation_y = msg->orientation.y;
    orientation_z = msg->orientation.z;
    orientation_w = msg->orientation.w;

    ROS_DEBUG("Compass Quaternions: %.1f\t%.1f\t%.1f\t%.1f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
} // end os5000Callback()


/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void StingrayModel::configCallback(stingray_model::stingray_modelConfig &config, uint32_t level)
{
    path_depth  = config.path_depth;
    path_radius = config.path_radius;
} // end configCallback()


/*---------------------------------------------------------------------
* main()
* Main function for ROS node.
* -------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "stingray_model");
    ros::NodeHandle n;

    // Create a StingrayModel object.
    StingrayModel *stingray_model = new StingrayModel();

    // Declare variables.
    int mode = 1;
    string gazebo_model_name;
    string pub_gazebo_name;
    string sub_ms3dmg_topic;
    string sub_os5000_topic;
    int rate = 40;

    // Robot state variables.
    double angle = 0;

    // Message declarations.
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    tf::TransformBroadcaster broadcaster;

    // Set up a dynamic reconfigure server before reading parameter server values.
    dynamic_reconfigure::Server<stingray_model::stingray_modelConfig> reconfig_srv;
    dynamic_reconfigure::Server<stingray_model::stingray_modelConfig>::CallbackType f;
    f = boost::bind(&StingrayModel::configCallback, stingray_model, _1, _2);
    reconfig_srv.setCallback(f);

    // Initialize node parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("rate",              rate,                        int(20));
    private_node_handle_.param("mode",              mode,                        int(1));
    private_node_handle_.param("gazebo_model_name", gazebo_model_name,           string("stingray"));
    private_node_handle_.param("pub_gazebo_name",   pub_gazebo_name,             string("gazebo/set_model_state"));
    private_node_handle_.param("path_depth",        stingray_model->path_depth , double(1.0));
    private_node_handle_.param("path_radius",       stingray_model->path_radius, double(1.0));
    private_node_handle_.param("sub_ms3dmg_topic",  sub_ms3dmg_topic,            string("mstrain_data"));
    private_node_handle_.param("sub_os5000_topic",  sub_os5000_topic,            string("os5000_data"));

    // Gazebo message.
    gazebo::ModelState gazebo_msg;
    gazebo_msg.reference_frame = "world";
    gazebo_msg.model_name = gazebo_model_name.c_str();

    // Set up ROS subscribers and clients so that data can be found.
    ros::Subscriber sub_ms3dmg = n.subscribe(sub_ms3dmg_topic.c_str(), 1, &StingrayModel::ms3dmgCallback, stingray_model);
    ros::Subscriber sub_os5000 = n.subscribe(sub_os5000_topic.c_str(), 1, &StingrayModel::os5000Callback, stingray_model);
    ros::Publisher pub_gazebo = n.advertise<gazebo::ModelState>(pub_gazebo_name.c_str(), 1);

    // Tell ROS how fast to run this node.
    ros::Rate loop_rate(rate);

    // Main loop.
    while (ros::ok())
    {
        // Update transform.
        // (Moving in a circle with radius = path_radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0.0; //cos(angle) * stingray_model->path_radius;
        odom_trans.transform.translation.y = 0.0; //sin(angle) * stingray_model->path_radius;
        odom_trans.transform.translation.z = 0.0; //stingray_model->path_depth;
        odom_trans.transform.rotation.x = stingray_model->orientation_x;
        odom_trans.transform.rotation.y = stingray_model->orientation_y;
        odom_trans.transform.rotation.z = stingray_model->orientation_z;
        odom_trans.transform.rotation.w = stingray_model->orientation_w;
        if (mode == 1)
        {
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle + M_PI / 2);
        }

        // Send the odometry transform.
        broadcaster.sendTransform(odom_trans);

        // Publish state information to Gazebo.
        gazebo_msg.pose.position.x    = odom_trans.transform.translation.x;
        gazebo_msg.pose.position.y    = odom_trans.transform.translation.y;
        gazebo_msg.pose.position.z    = odom_trans.transform.translation.z;
        gazebo_msg.pose.orientation.x = odom_trans.transform.rotation.x;
        gazebo_msg.pose.orientation.y = odom_trans.transform.rotation.y;
        gazebo_msg.pose.orientation.z = odom_trans.transform.rotation.z;
        gazebo_msg.pose.orientation.w = odom_trans.transform.rotation.w;
        pub_gazebo.publish(gazebo_msg);

        // Update the angle variable.
        if (rate > 0)
        {
            angle += M_PI / 180. / (rate / 10);
        }
        else
        {
            angle = 0.0;
        }

        // Let ROS sleep to keep proper loop rate.
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} // end main()
