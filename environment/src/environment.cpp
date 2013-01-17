/*------------------------------------------------------------------------------
 *  Title:       environment.cpp
 *  Description: ROS node for simulating an environment
 *
 * 				 Subscribes to Predicted future state from state estimator
 *
 * 				 Adds disturbances to state via dynamic_reconfiguration
 *
 * 				 Converts states to sensor readings
 *
 * 				 Adds electrical noise
 *
 * 				 Contains a map of the environment including location of obstacles
 *
 * 				 Publishes simulated sensor readings
 *
 * 				 NOTE: MAPPING FROM HEAVE SURGE SWAY, AND ROLL PITCH YAW
 * 				 TO XYZ, NEEDS TO BE DETERMINED
 * 				 FOR NOW, LINEAR:  SURGE=X,SWAY=Y,HEAVE=Z
 * 						  ANGULAR: ROLL=X,PITCH=Y,YAW=Z
 *----------------------------------------------------------------------------*/

#include "environment.h"

/*------------------------------------------------------------------------------
 * Environment()
 * Constructor.
 *----------------------------------------------------------------------------*/

Environment::Environment()
{
    InitEnvironment();
} // end Environment()


/*------------------------------------------------------------------------------
 * ~Environment()
 * Destructor.
 *----------------------------------------------------------------------------*/

Environment::~Environment()
{
} // end ~Environment()


/*---------------------------------------------------------------------
* InitEnvironment()
* Initializes environment
* -------------------------------------------------------------------*/

void Environment::InitEnvironment()
{

    // Initialize Earth Fixed Position
    lat = 0.;
    lon = 0.;
    depth = 0.;

    // Initialize Euler angles.
    roll = 0.;
    pitch = 0.;
    yaw = 0.;

    // Initialize Linear Velocities
    surge = 0.;
    sway = 0.;
    heave = 0.;

    // Initialize Quaternions.
    orientationX = 0.;
    orientationY = 0.;
    orientationZ = 0.;
    orientationW = 0.;

    // Initialize Linear Accelerations
    surge_dot = 0.;
    sway_dot = 0.;
    heave_dot = 0.;

    // Initialize Earth Fixed Angles from Magnetometer
    mag_x = 0.;
    mag_y = 0.;
    mag_z = 0.;

    ocean_depth = 1000.; //meters, not used now

    sea_floor_height = 0.; // meters above ocean depth, i.e. altitude = ocean_depth-depth-sea_floor_height

    //xyz + width + height + depth (location and size of an obstacle)
    //initialize to point mass at origin of earth centered coordinate frame
    for (int i = 0; i < 6; i++)
    {
        obstacle1[i] = 0.;
    }

    //Init noise scaling factor
    noiseScale = .001;

} //end InitEnvironment()


/*---------------------------------------------------------------------
* PublishSimulatedMicrostrainData()
* Publishes simulated Microstrain IMU data.
* -------------------------------------------------------------------*/

void Environment::PublishSimulatedMicrostrainData(ros::Publisher *_pubSimEuler)
{
    sensor_msgs::Imu msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;

    _pubSimEuler->publish(msg);
} // end publishSimulatedMicrostrainData()


/*---------------------------------------------------------------------
* PublishSimulatedDVLData()
* Publishes simulated Doppler Velocity Logger (DVL) data.
* -------------------------------------------------------------------*/

void Environment::PublishSimulatedDVLData(ros::Publisher *pubSimDVL)
{
    nav_msgs::Odometry msg;

    msg.twist.twist.linear.x = surge;
    msg.twist.twist.linear.y = sway;
    msg.twist.twist.linear.z = heave;
    msg.pose.pose.position.z = ocean_depth - sea_floor_height;

    pubSimDVL->publish(msg);
}// end publishSimulatedDVLData()


/*---------------------------------------------------------------------
* StateEstimatorCallback()
* Callback for new data.
* -------------------------------------------------------------------*/

void Environment::StateEstimatorCallback(const nav_msgs::Odometry::ConstPtr& _msg)
{
    lat = _msg->pose.pose.position.x;
    lon = _msg->pose.pose.position.y;
    depth = _msg->pose.pose.position.z + noiseScale * ((rand() / (float)RAND_MAX) - 0.5);
    if (depth < 0.)
    {
        depth = 0.;
    }

    // Convert quaternion to Euler angles.
    tfScalar roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(_msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //TODO: Make noise source Gaussian in stead of uniform
    roll  = (roll  + disturbanceRoll)  + noiseScale * ((rand() / (float)RAND_MAX) - 0.5);
    pitch = (pitch + disturbancePitch) + noiseScale * ((rand() / (float)RAND_MAX) - 0.5);
    yaw   = (yaw   + disturbanceYaw)   + noiseScale * ((rand() / (float)RAND_MAX) - 0.5);

    surge = (_msg->twist.twist.linear.x + disturbanceSpeed) + noiseScale * ((rand() / (float)RAND_MAX) - 0.5);
    sway  = (_msg->twist.twist.linear.y + disturbanceSway)  + noiseScale * ((rand() / (float)RAND_MAX) - 0.5);
    heave = (_msg->twist.twist.linear.z + disturbanceDepth) + noiseScale * ((rand() / (float)RAND_MAX) - 0.5);

    //TODO: Add these values back in when the appropriate standard message type is found.
    //surge_dot = _msg->surge_dot;
    //sway_dot  = _msg->sway_dot;
    //heave_dot = _msg->heave_dot;

    //mag_x = _msg->mag_x;
    //mag_y = _msg->mag_y;
    //mag_z = _msg->mag_z;
} // end StateEstimatorCallback()


/*---------------------------------------------------------------------
* DisturbanceCallback()
* Callback for new data.
* -------------------------------------------------------------------*/

void Environment::DisturbanceCallback(environment::environmentParamsConfig &_config, uint32_t _level)
{
    ROS_INFO("Reconfiguring Roll Disturbance to %f", _config.disturbanceRoll);
    disturbanceRoll = _config.disturbanceRoll;

    ROS_INFO("Reconfiguring Pitch Disturbance to %f", _config.disturbancePitch);
    disturbancePitch = _config.disturbancePitch;

    ROS_INFO("Reconfiguring Yaw Disturbance to %f", _config.disturbanceYaw);
    disturbanceYaw = _config.disturbanceYaw;

    ROS_INFO("Reconfiguring Depth Disturbance to %f", _config.disturbanceDepth);
    disturbanceDepth = _config.disturbanceDepth;

    ROS_INFO("Reconfiguring Speed Disturbance to %f", _config.disturbanceSpeed);
    disturbanceSpeed = _config.disturbanceSpeed;

    ROS_INFO("Reconfiguring Sway Disturbance to %f", _config.disturbanceSway);
    disturbanceSway = _config.disturbanceSway;
} // end DisturbanceCallback()


/*---------------------------------------------------------------------
* main()
* Main function.
* -------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "environment");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    // Declare variables.
    int rate;

    // Set up an Environment object.
    Environment *environment = new Environment();

    // Initialize node parameters.
    private_node_handle_.param("rate", rate, int(15));
    private_node_handle_.param("disturbanceRoll",  environment->disturbanceRoll, double(0.));
    private_node_handle_.param("disturbancePitch",  environment->disturbancePitch, double(0.));
    private_node_handle_.param("disturbanceYaw",  environment->disturbanceYaw, double(0.));
    private_node_handle_.param("disturbanceDepth",  environment->disturbanceDepth, double(0.));
    private_node_handle_.param("disturbanceSpeed",  environment->disturbanceSpeed, double(0.));
    private_node_handle_.param("disturbanceSway",  environment->disturbanceSway, double(0.));

    // Tell ROS to run this node at the desired rate.
    ros::Rate rEnv(rate);

    // Set up ROS subscribers and clients so that data can be found.
    ros::Subscriber state_prediction_sub = n.subscribe("state", 1, &Environment::StateEstimatorCallback, environment);

    // Set up ROS publisher for sensor signal emulation
    //! \todo, change this topic to ms3dmgSimData so when a real IMU is running, environment doesnt corrupt the data
    ros::Publisher pubSimMicrostrain = n.advertise<sensor_msgs::Imu>("ms3dmgData", 1000);
    ros::Publisher pubSimDVL = n.advertise<nav_msgs::Odometry>("simDVLData", 1000);

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<environment::environmentParamsConfig> disturbance_srv;
    dynamic_reconfigure::Server<environment::environmentParamsConfig>::CallbackType f = boost::bind(&Environment::DisturbanceCallback, environment, _1, _2);
    disturbance_srv.setCallback(f);

    // Main loop.
    while (n.ok())
    {
        //Publish the data
        environment->PublishSimulatedMicrostrainData(&pubSimMicrostrain);
        environment->PublishSimulatedDVLData(&pubSimDVL);
        //! \todo incorporate map
        //! Incorporate dyno control (subscribe to motor speed).
        ros::spinOnce();
        rEnv.sleep();
    }

    return 0;
} // end main()
