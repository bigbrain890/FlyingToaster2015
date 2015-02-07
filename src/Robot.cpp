#include "WPILib.h"
#include "math.h"
#include <sstream>
#define RIGHT_STICK_X_AXIS 2
#define LEFT_STICK_X_AXIS 0
#define LEFT_STICK_Y_AXIS 1


/**
 * Author: Ethan Novilla
 * Team: #3641 The Flying Toasters
 * Date: 1/3/2015
 *
 *
 * Overview: This program is made up for basic tank drive (with 4 omni wheels) and a H driver in the center that is
 * pnumatically actuated. Also, the AnalogChannel class been replaced with AnalogInput, and this is used
 * for the input of several analog sensors, like an ultrasonic sensor, and a potentiometer for position readout. It
 * is an ongoing project that will impliment more computation for
 */
class Robot: public SampleRobot
{
	RobotDrive chassis; 			// Object for robot movement.
	Joystick gamepad; 				// Logitech Dual Action for drive control.
	Joystick manipulator;   		// Object for later arm manipulator.
	AnalogInput ultraSonic; 		// Eventual ultrasonic sensor for distance measuring.
	AnalogInput lowerStagePot; 	  	// Potentiometer for positioning of lower forklift stage.
	AnalogInput upperStagePot;		// Potentiometer for positioning of upper forklift stage.
	Talon hDriveMotor;				// Splitter running 2 motors on H drive system.
	Talon lowerStage;				// Motor for running the lower stage of the fork lift.
	Talon upperStage;				// Motor for running the upper stage of the forck lift.
	Talon intakeMotor;				// Splitter running 2 motors on intake arm.
	DoubleSolenoid rightIntake;		// Actuating right intake armin into place.
	DoubleSolenoid leftIntake;		// Actuating left intake arm into place.
	PowerDistributionPanel pdBoard;	// pdBoard used for hardware debug
	std::ostringstream printDis;	// Object used to
	float distanceVoltage;			// voltage from ultrasonic.
	float distance;		  			// Distance in inches using scaler factor on distanceVoltage.
	float roundedDistance;			// Rounded number for inches to get rid of decimal places.
	float angle; 		  			// Value for potentiometer readout.
	float oldAngle = 0.0; 			// Used for de-bounce loop of pot readout.
	float upperLim;		  			// Used for de-bounce loop of pot readout.
	float lowerLim;					// Used for de-bounce loop of pot readout.


public:
	Robot() :
			chassis(0, 2, 1, 3),	// 4 PWM channels for control of drive motors.
			gamepad(0),				// Gamepad USB plug.
			manipulator(1),			// Manipulator USB plug.
			lowerStagePot (1), 		// Sensor feeding data into Analog input port 1.
			upperStagePot (2),		// Sensor feeding data into Analog input port 2.
			hDriveMotor(4),			// H drive motor.
			lowerStage(6),
			upperStage(7),
			intakeMotor(5),			// 2 motors feeding off of pwm output 5
			rightIntake(0),
			leftIntake(1)

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
			chassis.ArcadeDrive(gamepad.GetRawAxis(LEFT_STICK_Y_AXIS), gamepad.GetRawAxis(RIGHT_STICK_X_AXIS)*-1);
			while (gamepad.GetRawButton(6) == true)
			{
				chassis.ArcadeDrive(gamepad.GetRawAxis(LEFT_STICK_Y_AXIS), gamepad.GetRawAxis(RIGHT_STICK_X_AXIS)*-1);
				hDriveMotor.Set(gamepad.GetRawAxis(LEFT_STICK_X_AXIS)*-1);

			}
			hDriveMotor.Set(0);
			Wait(0.0005);
			distanceVoltage = ultraSonic.GetVoltage(); 						// Reading raw voltage from ultra sonic into variable distanceVoltage.
			distance = distanceVoltage / 0.00977;	   						// Converting voltage to inches with scaler.
			roundedDistance = ceilf(distance * 100) / 100; 					// Doing calculations to round distance value.
			SmartDashboard::PutString("DB/String 0", printDis.str().c_str());
			oldAngle = angle;		  										// Take the last angle and store it as the old angle.
			upperLim = oldAngle + 0.01;										// Set an upper limit to compare new angle to
			lowerLim = oldAngle - 0.01;										// Set a lower limit to compare new angle to
			if ((angle > upperLim) || (angle < lowerLim))					// If angle has actually changed a considerable amount...
			{
				SmartDashboard::PutNumber("Angle", angle);  				// Print it out to the dashboard.
			}
			if ((angle < upperLim) || (angle > lowerLim))
			{
				Wait(.0001);
			}
		}

	}

	void Test()
	{

#define PORT_COUNT 16
		double ports[PORT_COUNT];
		for (int i = 0; i < PORT_COUNT; i++)
			ports[i] = pdBoard.GetCurrent(i);
		for (int i = 0; i < PORT_COUNT; i++)
		{
			std::ostringstream strstrm;
			strstrm << "Current Draw " << i << ": ";
			SmartDashboard::PutNumber(strstrm.str().c_str(), ports[i]);
		}
	}
};

START_ROBOT_CLASS(Robot);
