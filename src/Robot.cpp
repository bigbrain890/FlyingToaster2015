#include "WPILib.h"

/**
 * This is the basic code for a mecanum drive robot. All it does is drive a macanum chassis. Field centric mecanum
 * is not supported as I'm not using a gyro for the 4th argument of the chassis object.
 */
class Robot: public SampleRobot
{
	RobotDrive chassis; // Object for robot movement.
	Joystick gamepad; // Logitech Dual Action for drive control.
	Joystick manipulator; // Object for later arm manipulator.
	float chassisGyro = 0.0; //Arbitrary gyro value for 4th float gyro value of mecanum cartesian drive

public:
	Robot() :
			chassis(0, 1, 2, 3),	// 4 PWM channels for control of drive motors.
			gamepad(0),		// Gamepad USB plug
			manipulator(1)	// Manipulator USB plug
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
		}
	}

	void Test()
	{
	}
};

START_ROBOT_CLASS(Robot);
