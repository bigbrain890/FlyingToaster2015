#include "WPILib.h"
#include "math.h"

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
	const float chassisGyro = 0.0; //Arbitrary gyro value for 4th float gyro value of mecanum cartesian drive
	float distanceVoltage; // voltage from ultrasonic
	float distance;		   // Distance in inches using scaler factor on distanceVoltage
	float roundedDistance; // Rounded number for inches to get rid of decimal places.
	float angle; // Value for potentiometer readout
	float oldAngle = 0.0;
	float upperLim;
	float lowerLim;

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
			chassis.SetInvertedMotor(chassis.kFrontLeftMotor, true); // Invert front left motor.
			chassis.SetInvertedMotor(chassis.kFrontRightMotor, true);// Invert front right motor.
			chassis.MecanumDrive_Cartesian(gamepad.GetRawAxis(1), gamepad.GetRawAxis(2), gamepad.GetRawAxis(3), chassisGyro);
			Wait(0.0005); // wait for a motor update time
			distanceVoltage = ultraSonic.GetVoltage(); // reading raw voltage from ultra sonic into variable distanceVoltage
			oldAngle = angle;		  // Take the last angle and store it as the old angle
			angle = pot.GetVoltage(); // read voltage in from pot and store into variable angle.
			distance = distanceVoltage / 0.00977;	   // converting voltage to inches with scaler
			roundedDistance = ceilf(distance * 100) / 100; // Doing calculations to round distance value.
			SmartDashboard::PutNumber("Distance", roundedDistance); // Print the distance to SmartDashboard (not so smart).
			upperLim = oldAngle + 0.01;		// Set an upper limit to compare new angle to
			lowerLim = oldAngle - 0.01;		// Set a lower limit to compare new angle to
			if ((angle > upperLim) || (angle < lowerLim))	// If angle has actually changed a considerable amount...
			{
				SmartDashboard::PutNumber("Angle", angle);  // Print it out to the dashboard.
			}
			if ((angle < upperLim) && (angle > lowerLim))   // If it hasn't...
			{
				Wait(0.005);								// Don't do anything
			}

		}

	}

	void Test()
	{
	}
};

START_ROBOT_CLASS(Robot);
