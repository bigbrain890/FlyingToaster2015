#include "WPILib.h"

/**
 * This is the basic code for a mecanum drive robot. All it does is drive a macanum chassis. Field centric mecanum
 * is not supported as I'm not using a gyro for the 4th argument of the chassis object. Also, the AnalogChannel class
 * been replaced with AnalogInput.
 */
class Robot: public SampleRobot
{
	RobotDrive chassis; // Object for robot movement.
	Joystick gamepad; // Logitech Dual Action for drive control.
	Joystick manipulator; // Object for later arm manipulator.
	AnalogInput ultraSonic; //eventual ultrasonic sensor for distance measuring
	AnalogInput pot; 		  // Potentiometer for positioning of eveutual forklift
	float chassisGyro = 0.0; //Arbitrary gyro value for 4th float gyro value of mecanum cartesian drive
	float distanceVoltage;
	float distance;

public:
	Robot() :
			chassis(0, 1, 2, 3),	// 4 PWM channels for control of drive motors.
			gamepad(0),				// Gamepad USB plug
			manipulator(1),			// Manipulator USB plug
			ultraSonic (0), 		// Sensor feeding data into Analog input port 0
			pot (1) 				//Sensor feeding data into Analog input port 1

	{
		chassis.SetExpiration(0.1);
	}


	void Autonomous()
	{

	}

	void OperatorControl()
	{
		chassis.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
		{
			chassis.MecanumDrive_Cartesian(gamepad.GetRawAxis(1), gamepad.GetRawAxis(2), gamepad.GetRawAxis(3), chassisGyro);
			Wait(0.005); // wait for a motor update time
			distanceVoltage = ultraSonic.GetVoltage(); // reading raw voltage from ultra sonic into variable distanceVoltage
			distance = distanceVoltage / 0.009766;	   // Converting raw voltage into inches with scaler.
		}
	}

	void Test()
	{
	}
};

START_ROBOT_CLASS(Robot);
