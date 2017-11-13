/*
This node allows to take manual control of the cameleon robot at any time.
When the robot is not controlled manually, the node simply receives commands over the cmd and flipperVelocity topic and passes them over to the cameleon/cmd_vel 
When the operator presses manualOverrideButton, the cmd and flipperVelocity messages are discarted and the operator can drive the robot manually. 
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>


/*joystick input parameters - which button causes the user to take control and axes that correspond to forward, turning and flipper speeds*/ 
int manualOverrideButton = 4;
int linearAxis = 1;
int angularAxis = 0;
int flipperAxis = 4;

/*these constants determine how quickly the robot moves based on the joystick input*/ 
double linearGain = 0.2;
double angularGain = 0.2;
double flipperGain = 0.2;

/*listening to joystick, flipperVelocity and cmd topics and publishing commands to the cameleon ros driver*/
ros::Publisher vel_pub_;
ros::Subscriber flipperSub;
ros::Subscriber joy_sub_;
ros::Subscriber cmd_sub_;

/*state variables - twist is the message that eventually gets to the ROS driver of the robot, other are obvious*/
geometry_msgs::Twist twist;
bool teleoperated = false;
double forwardSpeed = 0;
double forwardAcceleration= 0;
double flipperSpeed = 0;
int flipperValid = 0;

/*commands from higher-lever modules*/
void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	/*if the robot is note teleoperated, form the twist command from the incoming cmd topic*/
	if (teleoperated == false){
		 twist.linear.x  = cmd->linear.x;
		 twist.angular.z = cmd->angular.z;
		 if (flipperValid <= 0) twist.angular.y = cmd->angular.y;
	}
	if (--flipperValid < 0)	 flipperValid = 0;
}

/*flipper velocity can come from another component*/
void flipperCallback(const std_msgs::Float32::ConstPtr& msg)
{
	flipperValid = 10;
	if (teleoperated == false) twist.angular.y = msg->data;   
}

/*receiving joystick data*/
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{     
       	/*if swiching modes from teleoperated to manual and vice versa, then clear velocities*/
	if (teleoperated != joy->buttons[manualOverrideButton]){
		twist.angular.z = 0.0;
		twist.linear.x = 0.0;
		twist.angular.y = 0.0;
		forwardAcceleration = 0;
		forwardSpeed = 0;
	}

	/*is it teleoperated?*/
	teleoperated = (joy->buttons[manualOverrideButton] == 1);

	/*if yes, form the twist command from the joystick input*/
	if (teleoperated)
	{
		twist.angular.z = angularGain*joy->axes[angularAxis];
		forwardAcceleration = 0.02*joy->axes[linearAxis];;
		twist.angular.y = flipperGain*joy->axes[flipperAxis];
		ROS_INFO( "%i %i %i ",flipperAxis,angularAxis,linearAxis);
		ROS_INFO( "Rychlost z= %f, Rychlost x= %f ",  twist.linear.z,twist.linear.x);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cameleon_teleop");
	ros::NodeHandle nh;

	nh.param("axis_linear", linearAxis, 1);
	nh.param("axis_angular", angularAxis, 0);
	nh.param("axis_flipper", flipperAxis, 4);
	nh.param("manual_override_button", manualOverrideButton, 4);

	nh.param("scale_angular", angularGain, 0.2);
	nh.param("scale_linear", linearGain, 0.2);
	nh.param("scale_flipper", flipperGain, 0.2);

	vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_twist", 1);
	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
	cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd", 10, cmdCallback);
	flipperSub = nh.subscribe("/flipperVelocity", 1, flipperCallback);

	while (ros::ok()){
		ros::spinOnce();
		if (teleoperated){
			forwardSpeed += forwardAcceleration;
			forwardSpeed = fmin(fmax(forwardSpeed,-1.0),1.0);
			twist.linear.x =  forwardSpeed*linearGain;;
		}
		vel_pub_.publish(twist);
		usleep(50000);
	}
}
