/*------------------------------------------------------------------------------
 *  Title:        nav.cpp
 *  Description:  ROS node for running Navigation.
 *----------------------------------------------------------------------------*/

#include "nav.h"

/*------------------------------------------------------------------------------
 * Nav()
 * Constructor.
 *----------------------------------------------------------------------------*/

Nav::Nav()
{
    initStates();
} // end Nav()


/*------------------------------------------------------------------------------
 * ~Nav()
 * Destructor.
 *----------------------------------------------------------------------------*/

Nav::~Nav()
{
} // end ~Nav()


/*---------------------------------------------------------------------
* initStates()
* Initializes measured states, target states, and errors to zero.
* -------------------------------------------------------------------*/

void Nav::initStates()
{
    // Euler angles.
    roll = 0.;
    pitch = 0.;
    yaw = 0.;
    depth = 0.;
    surge = 0.;

    // Initialize Targets
    target_roll = 0.;
    target_pitch = 0.;
    target_yaw = 0.;
    target_depth = 0.;
    target_surge = 0.;

    // All control done in Euler angle frame for now (targets in RPY).
    // Init previous errors for differentiation.
    prev_roll_error = 0.;
    prev_pitch_error = 0.;
    prev_yaw_error = 0.;
    prev_depth_error = 0.;
    prev_surge_error = 0.;

    // Init previous integrator values.
    prev_roll_int = 0.;
    prev_pitch_int = 0.;
    prev_yaw_int = 0.;
    prev_depth_int = 0.;
    prev_surge_int = 0.;

    // Init previous derivative calues
    prev_roll_deriv = 0.;
    prev_pitch_deriv = 0.;
    prev_yaw_deriv = 0.;
    prev_depth_deriv = 0.;
    prev_surge_deriv = 0.;

    // Init alpha terms
    alpha_roll = 0.;
    alpha_pitch = 0.;
    alpha_yaw = 0.;
    alpha_depth = 0.;
    alpha_surge = 0.;


    // Init variables for storing current integrator values.
    // NOT USED???? PERRY CANT FIND
    curr_pitch_int = 0.;
    curr_roll_int = 0.;
    curr_yaw_int = 0.;
    curr_depth_int = 0.;
    curr_surge_int = 0.;

    // Initialize all control outputs to zero.
    u_roll = 0.;
    u_pitch = 0.;
    u_yaw = 0.;
    u_depth = 0.;
    u_surge = 0.;

    // Initialize actuator constraints - vehicle specific, should load from config file at some point.
    //max yaw
    max_yaw_angle = 180.0;
    // Rudder is symmetric, min = -max.
    min_yaw_angle = -1 * max_yaw_angle;

    // Initialize delta time for each control loop.
    dt_roll = 0.;
    dt_pitch = 0.;
    dt_yaw = 0.;
    dt_depth = 0.;
    dt_surge = 0.;

    roll_upsidedown = false;
} //end initStates()


/*------------------------------------------------------------------------------
 * publishControlInputs()
 * Publishes control input data.
 *----------------------------------------------------------------------------*/

void Nav::publishControlInputs(ros::Publisher *pub_control_inputs)
{
    nav::ControlInputs msg;

    msg.u_roll  = u_roll;
    msg.u_pitch = u_pitch;
    msg.u_yaw   = u_yaw;
    msg.u_depth = u_depth;
    msg.u_surge = u_surge;

    pub_control_inputs->publish(msg);
} // end publishControlInputs()


/*------------------------------------------------------------------------------
 * microstrainCallback()
 * Callback for IMU data.
 *----------------------------------------------------------------------------*/

void Nav::microstrainCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Convert quaternion to RPY.
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  
    if (roll>0)
        roll = 3.14159-roll;
    else
        roll = -3.14159-roll;

   //Perry Hack
    double temp = pitch;
    pitch = -roll*180/M_PI;
    roll = -temp*180/M_PI;
    yaw = yaw*180/M_PI;
    
    // Gabe
    // Correction of data code. Take roll and flip it
    if (roll_upsidedown)
    {
		if (roll > 0)
		{
			roll -= M_PI;
		}
		else
		{
			roll += M_PI;
		}
	}
    
    ROS_DEBUG("Microstrain RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
    //ROS_DEBUG_THROTTLE(15, "Microstrain RPY = (%lf, %lf, %lf)", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
    //ROS_DEBUG("Microstrain Quaternions = (%lf, %lf, %lf, %lf)", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    ROS_DEBUG("Microstrain RPY = (%lf, %lf, %lf)", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
    ROS_DEBUG("Microstrain Quaternions = (%lf, %lf, %lf, %lf)", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
} // end microstrainCallback()


/*------------------------------------------------------------------------------
 * compassCallback()
 * Callback for compass data.
 *----------------------------------------------------------------------------*/

void Nav::compassCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Convert quaternion to RPY.
    /* TODO: get depth from compass
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_DEBUG("OS5000 RPY = (%lf, %lf, %lf)", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
    */
} // end compassCallback()


/*------------------------------------------------------------------------------
 * targetStatesCallback()
 * Callback for targets.
 *----------------------------------------------------------------------------*/

void Nav::targetStatesCallback(const planner::TargetStates::ConstPtr& msg)
{
    target_roll  = msg->target_roll;
    target_pitch = msg->target_pitch;
    target_yaw   = msg->target_yaw;
    target_depth = msg->target_depth;
    target_surge = msg->target_surge;

    ROS_DEBUG("Nav: Target States Received target roll = %f , target pitch = %f , target yaw = %f , target depth = %f , target surge = %f", target_roll, target_pitch, target_yaw, target_depth, target_surge);
} // end targetStatesCallback()

void Nav::compassDepthCallback( const os5000::DepthMessage::ConstPtr& msg)
{
	depth = msg->depth;
	ROS_DEBUG("DEPTH: %lf", depth);

}
/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void Nav::configCallback(nav::navParamsConfig& config, uint32_t level)
{
    ROS_INFO("Reconfiguring Roll to : (P, I, D) = (%f %f %f)", config.gain_roll_p, config.gain_roll_i, config.gain_roll_d);
    gain_roll_p = config.gain_roll_p;
    gain_roll_i = config.gain_roll_i;
    gain_roll_d = config.gain_roll_d;
    alpha_roll=config.alpha_roll;
    min_int_roll = config.min_int_roll;
    max_int_roll = config.max_int_roll;

    ROS_INFO("Reconfiguring Pitch to : (P, I, D) = (%f %f %f)", config.gain_pitch_p, config.gain_pitch_i, config.gain_pitch_d);
    gain_pitch_p = config.gain_pitch_p;
    gain_pitch_i = config.gain_pitch_i;
    gain_pitch_d = config.gain_pitch_d;
    alpha_pitch=config.alpha_pitch;
    min_int_pitch = config.min_int_pitch;
    max_int_pitch = config.max_int_pitch;

    ROS_INFO("Reconfiguring Yaw to : (P, I, D) = (%f %f %f)", config.gain_yaw_p, config.gain_yaw_i, config.gain_yaw_d);
    gain_yaw_p = config.gain_yaw_p;
    gain_yaw_i = config.gain_yaw_i;
    gain_yaw_d = config.gain_yaw_d;
    alpha_yaw=config.alpha_yaw;
    min_int_yaw = config.min_int_yaw;
    max_int_yaw = config.max_int_yaw;

    ROS_INFO("Reconfiguring Depth to : (P, I, D) = (%f %f %f)", config.gain_depth_p, config.gain_depth_i, config.gain_depth_d);
    gain_depth_p = config.gain_depth_p;
    gain_depth_i = config.gain_depth_i;
    gain_depth_d = config.gain_depth_d;
    alpha_depth=config.alpha_depth;
    min_int_depth = config.min_int_depth;
    max_int_depth = config.max_int_depth;

    ROS_INFO("Reconfiguring Surge to : (P, I, D) = (%f %f %f)", config.gain_surge_p, config.gain_surge_i, config.gain_surge_d);
    gain_surge_p = config.gain_surge_p;
    gain_surge_i = config.gain_surge_i;
    gain_surge_d = config.gain_surge_d;
    alpha_surge=config.alpha_surge;
    min_int_surge = config.min_int_surge;
    max_int_surge = config.max_int_surge;
    
    roll_upsidedown = config.roll_upsidedown;
} // end configCallback()


/*------------------------------------------------------------------------------
 * main()
 * Main function.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "nav");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    // Declare variables.
    int rate;
    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    double dt = 0.;

    // Set up a Nav object.
    Nav *nav = new Nav();

    // Initialize node parameters.
    private_node_handle_.param("rate", rate, int(100));
    private_node_handle_.param("gain_roll_p",   nav->gain_roll_p, double(6.));
    private_node_handle_.param("gain_roll_i",   nav->gain_roll_i, double(0.));
    private_node_handle_.param("gain_roll_d",   nav->gain_roll_d, double(0.));
    private_node_handle_.param("gain_pitch_p",  nav->gain_pitch_p, double(6.));
    private_node_handle_.param("gain_pitch_i",  nav->gain_pitch_i, double(0.));
    private_node_handle_.param("gain_pitch_d",  nav->gain_pitch_d, double(0.));
    private_node_handle_.param("gain_yaw_p",    nav->gain_yaw_p, double(0.5));
    private_node_handle_.param("gain_yaw_i",    nav->gain_yaw_i, double(0.));
    private_node_handle_.param("gain_yaw_d",    nav->gain_yaw_d, double(0.0));
    private_node_handle_.param("gain_depth_p",  nav->gain_depth_p, double(1.));
    private_node_handle_.param("gain_depth_i",  nav->gain_depth_i, double(0.));
    private_node_handle_.param("gain_depth_d",  nav->gain_depth_d, double(0.));
    private_node_handle_.param("gain_surge_p",  nav->gain_surge_p, double(10.));
    private_node_handle_.param("gain_surge_i",  nav->gain_surge_i, double(0.));
    private_node_handle_.param("gain_surge_d",  nav->gain_surge_d, double(0.));
    private_node_handle_.param("target_roll",   nav->target_roll, double(0.));
    private_node_handle_.param("target_pitch",  nav->target_pitch, double(0.));
    private_node_handle_.param("target_yaw",    nav->target_yaw, double(0.));
    private_node_handle_.param("target_depth",  nav->target_depth, double(0.));
    private_node_handle_.param("target_surge",  nav->target_surge, double(0.));
    private_node_handle_.param("min_int_pitch", nav->min_int_pitch, double(0.));
    private_node_handle_.param("max_int_pitch", nav->max_int_pitch, double(0.));
    private_node_handle_.param("min_int_roll",  nav->min_int_roll, double(0.));
    private_node_handle_.param("max_int_roll",  nav->max_int_roll, double(0.));
    private_node_handle_.param("min_int_yaw",   nav->min_int_yaw, double(0.));
    private_node_handle_.param("max_int_yaw",   nav->max_int_yaw, double(0.));
    private_node_handle_.param("min_int_depth", nav->min_int_depth, double(0.));
    private_node_handle_.param("max_int_depth", nav->max_int_depth, double(0.));
    private_node_handle_.param("min_int_surge", nav->min_int_surge, double(0.));
	private_node_handle_.param("max_int_surge", nav->max_int_surge, double(0.));    
	private_node_handle_.param("alpha_roll", nav->alpha_roll, double(0.));
    private_node_handle_.param("alpha_pitch", nav->alpha_pitch, double(0.));
    private_node_handle_.param("alpha_yaw", nav->alpha_yaw, double(0.));
    private_node_handle_.param("alpha_depth", nav->alpha_depth, double(0.));
    private_node_handle_.param("alpha_surge", nav->alpha_surge, double(0.));


    // Tell ROS to run this node at the desired rate.
    ros::Rate loop_rate(rate);

    // Set up ROS subscribers and clients so that data can be found.
    
    // !! Removed subscriptions to compass and planner nodes
    // ros::Subscriber compass_sub = n.subscribe("os5000_data", 1000, &Nav::compassCallback, nav);
       ros::Subscriber compass_depth = n.subscribe("depthMessage", 1000, &Nav::compassDepthCallback, nav);
    ros::Subscriber microstrain_sub = n.subscribe("mstrain_data", 1000, &Nav::microstrainCallback, nav);
    ros::Subscriber target_states_sub = n.subscribe("targetStates", 1000, &Nav::targetStatesCallback, nav);
    ros::ServiceClient pid_client = n.serviceClient<pid::PID>("PID");
    pid::PID pid_srv;

    // Set up ROS publisher for control inputs
    ros::Publisher pub_control_inputs = n.advertise<nav::ControlInputs>("controlInputs", 1000);

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<nav::navParamsConfig> gain_srv;
    dynamic_reconfigure::Server<nav::navParamsConfig>::CallbackType f;
    f = boost::bind(&Nav::configCallback, nav, _1, _2);
    gain_srv.setCallback(f);

    // Set the time difference.
    nav->dt_roll  = 0.0;
    nav->dt_pitch = 0.0;
    nav->dt_yaw   = 0.0;
    nav->dt_depth = 0.0;
    nav->dt_surge = 0.0;

    // Main loop.
    while (n.ok())
    {
        // Calculate dt.
        current_time = ros::Time::now();
        dt = current_time.toSec() - last_time.toSec();
        ROS_DEBUG("dt = %lf", dt);

        //////////////////////
        // Roll controller. //
        //////////////////////
        // Call PID server.
        pid_srv.request.current_val = nav->roll;

        // Set target values.
        //! \todo Add message from planner to provide targets, for now set to 0.
        pid_srv.request.target_val = nav->target_roll;

        // Send previous errors, gains and integrator values to PID.
        pid_srv.request.previous_error = nav->prev_roll_error;
        pid_srv.request.previous_integrator_val = nav->prev_roll_int;
        pid_srv.request.previous_derivative_val = nav->prev_roll_deriv;
	    pid_srv.request.alpha = nav->alpha_roll;
	    nav->dt_roll = dt;
        pid_srv.request.dt = nav->dt_roll;
        pid_srv.request.gain_p = nav->gain_roll_p;
        pid_srv.request.gain_i = nav->gain_roll_i;
        pid_srv.request.gain_d = nav->gain_roll_d;
        pid_srv.request.integral_term_min = nav->min_int_roll;
        pid_srv.request.integral_term_max = nav->max_int_roll;

        ROS_DEBUG("Roll = %f", nav->roll);
        // Call PID service.
        if (pid_client.call(pid_srv))
        {
            nav->u_roll = pid_srv.response.u;
            nav->prev_roll_int = pid_srv.response.current_integrator_val;
	        nav->prev_roll_deriv=pid_srv.response.current_derivative_val;
            nav->prev_roll_error = pid_srv.response.current_error;
            ROS_DEBUG("U Roll = %f", pid_srv.response.u);
        }
        else
        {
            ROS_ERROR("Failed to call PID service for roll.");
        }

        ///////////////////////
        // Pitch controller. //
        ///////////////////////
        // Call PID server.
        pid_srv.request.current_val = nav->pitch;

        // Set target values.
        //! \todo Add message from planner to provide targets, for now set to 0.
        pid_srv.request.target_val = nav->target_pitch;

        // Send previous errors, gains and integrator values to PID.
        pid_srv.request.previous_error = nav->prev_pitch_error;
        pid_srv.request.previous_integrator_val = nav->prev_pitch_int;
		pid_srv.request.previous_derivative_val = nav->prev_pitch_deriv;
		pid_srv.request.alpha = nav->alpha_pitch;       
 		nav->dt_pitch = dt;
        pid_srv.request.dt = nav->dt_pitch;
        pid_srv.request.gain_p = nav->gain_pitch_p;
        pid_srv.request.gain_i = nav->gain_pitch_i;
        pid_srv.request.gain_d = nav->gain_pitch_d;
        pid_srv.request.integral_term_min = nav->min_int_pitch;
        pid_srv.request.integral_term_max = nav->max_int_pitch;

        ROS_DEBUG("Pitch = %f", nav->pitch);
        ROS_DEBUG("Pitch Proportional Gain = %f", nav->gain_pitch_p);
        // Call PID service.
        if (pid_client.call(pid_srv))
        {
            nav->u_pitch = pid_srv.response.u;
            nav->prev_pitch_int = pid_srv.response.current_integrator_val;
	    	nav->prev_pitch_deriv = pid_srv.response.current_derivative_val;
            nav->prev_pitch_error = pid_srv.response.current_error;
            ROS_DEBUG("U Pitch = %f", pid_srv.response.u);
        }
        else
        {
            ROS_ERROR("Failed to call PID service for pitch.");
        }

        /////////////////////
        // Yaw controller. //
        /////////////////////
        // Make sure we are choosing the smaller angle as our error
        // i.e. current = 0, target = 90, dont turn -270 to get there.
        // If PID ever changes order of error, these signs will change.
        // Currently based on err = target - actual.

        // Temp fix: if the vehicle has a rudder in the rear, we need
        // to multiply by negative 1 to reflect positive control
        // surface angle moves vehicle in negative (ccw) direction
        // This property has been accounted for in the vehicle model
        // (see stateEstimator), so I am making the control match
        // that here. In near future, need to have model  and model
        // params in a cfg file, and somehow tell the controller to
        // switch the sign for u.

        // Wrap current value by -360 to bring error within 180.
        if (nav->target_yaw - nav->yaw < -180.0)
        {
            pid_srv.request.current_val = -1.0 * (nav->yaw - 2 * 180.0);
        }
        else if (nav->target_yaw - nav->yaw > 180.0)
        {
            // Wrap current value by +360 to bring error within 180.
            pid_srv.request.current_val = -1.0 * (nav->yaw + 2 * 180.0);
        }
        else
        {
            pid_srv.request.current_val = -1.0 * nav->yaw;
        }

        // Set target values. See note above for -1.0 explanation
        pid_srv.request.target_val = -1.0 * nav->target_yaw;

        // Send previous errors, gains and integrator values to PID.
        pid_srv.request.previous_error = nav->prev_yaw_error;
        pid_srv.request.previous_integrator_val = nav->prev_yaw_int;
		pid_srv.request.previous_derivative_val= nav->prev_yaw_deriv;
		pid_srv.request.alpha = nav->alpha_yaw;               
		nav->dt_yaw = dt;
        pid_srv.request.dt = nav->dt_yaw;
        pid_srv.request.gain_p = nav->gain_yaw_p;
        pid_srv.request.gain_i = nav->gain_yaw_i;
        pid_srv.request.gain_d = nav->gain_yaw_d;
        pid_srv.request.integral_term_min = nav->min_int_yaw;
        pid_srv.request.integral_term_max = nav->max_int_yaw;

        ROS_DEBUG("Yaw = %f", nav->yaw);
        // Call PID service.
        if (pid_client.call(pid_srv))
        {
            nav->u_yaw = pid_srv.response.u;

            //inforce bounds on control surface angle command
            if (nav->u_yaw > nav->max_yaw_angle)
            {
                nav->u_yaw = nav->max_yaw_angle;
            }
            if (nav->u_yaw < nav->min_yaw_angle)
            {
                nav->u_yaw = nav->min_yaw_angle;
            }
            nav->prev_yaw_int = pid_srv.response.current_integrator_val;
	    	nav->prev_yaw_deriv = pid_srv.response.current_derivative_val;
            nav->prev_yaw_error = pid_srv.response.current_error;
            ROS_DEBUG("U Yaw = %f", pid_srv.response.u);
        }
        else
        {
            ROS_ERROR("Failed to call PID service for yaw.");
        }

        ///////////////////////
        // Depth controller. //
        ///////////////////////
        // Call PID server.
        pid_srv.request.current_val = nav->depth;

        // Set target values.
        //! \todo Add message from planner to provide targets, for now set to 0.
        pid_srv.request.target_val = nav->target_depth;

        // Send previous errors, gains and integrator values to PID.
        pid_srv.request.previous_error = nav->prev_depth_error;
        pid_srv.request.previous_integrator_val = nav->prev_depth_int;
		pid_srv.request.previous_derivative_val = nav->prev_depth_deriv;
		pid_srv.request.alpha = nav->alpha_depth;
        nav->dt_depth = dt;
        pid_srv.request.dt = nav->dt_depth;
        pid_srv.request.gain_p = nav->gain_depth_p;
        pid_srv.request.gain_i = nav->gain_depth_i;
        pid_srv.request.gain_d = nav->gain_depth_d;
        pid_srv.request.integral_term_min = nav->min_int_depth;
        pid_srv.request.integral_term_max = nav->max_int_depth;

        ROS_DEBUG("Depth = %f", nav->depth);
        // Call PID service.
        if (pid_client.call(pid_srv))
        {
            nav->u_depth = pid_srv.response.u;
            nav->prev_depth_int = pid_srv.response.current_integrator_val;
		    nav->prev_depth_deriv = pid_srv.response.current_derivative_val;
            nav->prev_depth_error = pid_srv.response.current_error;
            ROS_DEBUG("U Depth = %f", pid_srv.response.u);
        }
        else
        {
            ROS_ERROR("Failed to call PID service for depth.");
        }

        ///////////////////////
        // Surge controller. //
        ///////////////////////
        // Call PID server.
        pid_srv.request.current_val = nav->surge;

        // Set target values.
        //! \todo Add message from planner to provide targets, for now set to 0.
        pid_srv.request.target_val = nav->target_surge;

        // Send previous errors, gains and integrator values to PID.
        pid_srv.request.previous_error = nav->prev_surge_error;
        pid_srv.request.previous_integrator_val = nav->prev_surge_int;
		pid_srv.request.previous_derivative_val = nav->prev_surge_deriv;
        nav->dt_surge = dt;
        pid_srv.request.dt = nav->dt_surge;
        pid_srv.request.gain_p = nav->gain_surge_p;
        pid_srv.request.gain_i = nav->gain_surge_i;
        pid_srv.request.gain_d = nav->gain_surge_d;
        pid_srv.request.integral_term_min = nav->min_int_surge;
        pid_srv.request.integral_term_max = nav->max_int_surge;

        ROS_DEBUG("Surge = %f", nav->surge);
        // Call PID service.
        if (pid_client.call(pid_srv))
        {
            nav->u_surge = pid_srv.response.u;
            nav->prev_surge_int = pid_srv.response.current_integrator_val;
	   		nav->prev_surge_deriv = pid_srv.response.current_derivative_val;
            nav->prev_surge_error = pid_srv.response.current_error;
            ROS_DEBUG("U Surge = %f", pid_srv.response.u);
        }
        else
        {
            ROS_ERROR("Failed to call PID service for surge.");
        }

        // Publish the control values so the motors can use them.
        nav->publishControlInputs(&pub_control_inputs);

        // Reset last_time for calculation of next dt.
        last_time = current_time;

        ros::spinOnce();

        loop_rate.sleep();
        //ros::Duration(2).sleep();
    }

    return 0;
} // end main()
