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
	AnalogInput lowerStagePot; 	  	// Potentiometer for positioning of lower forklift stage.
	AnalogInput upperStagePot;		// Potentiometer for positioning of upper forklift stage.
	Encoder leftDriveEncoder;
	Encoder rightDriveEncoder;
	Encoder hDrive;
	Talon hDriveMotor;				// Splitter running 2 motors on H drive system.
	Talon lowerStage;				// Motor for running the lower stage of the fork lift.
	Talon upperStage;				// Motor for running the upper stage of the fork lift.
	Talon intakeMotor;				// Splitter running 2 motors on intake arm.
	DoubleSolenoid rightIntake;		// Actuating right intake arm into place.
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
	float Proportion = 0.9;         // This is the P in PID, a.k.a proportion.   !!!!TUNE THIS FOR CHANGE IN ACCELERATION!!!!
	double upperLiftMax = 0;
	double upperLiftMin = 0;
	double lowerLiftMax = 0;
	double lowerLiftMin = 0;
	float LoadSetPoint = 1.5;         // This integer is used as the PID Loops set-point.
	double PickUpSetPoint = 1;


public:
	Robot() :
			chassis(0, 2, 1, 3),	// 4 PWM channels for control of drive motors.
			gamepad(0),				// Gamepad USB plug.
			manipulator(1),			// Manipulator USB plug.
			lowerStagePot (1), 		// Sensor feeding data into Analog input port 1.
			upperStagePot (2),		// Sensor feeding data into Analog input port 2.
			leftDriveEncoder(2,3),
			rightDriveEncoder(0,1),
			hDrive(4,5),
			hDriveMotor(4),			// H drive motor.
			lowerStage(6),
			upperStage(7),
			intakeMotor(5),		// 2 motors feeding off of pwm output 5
			rightIntake(0, 1),
			leftIntake(2,3)



	{
		chassis.SetExpiration(0.1);
	}


	void Autonomous()
	{
		leftDriveEncoder.SetDistancePerPulse(0.076199111842);
		rightDriveEncoder.SetDistancePerPulse(0.076199111842);
		double distance = 0;
		bool button1 = SmartDashboard::GetBoolean("DB/Button 0", false);
		bool button2 = SmartDashboard::GetBoolean("DB/Button 1", false);
		bool button3 = SmartDashboard::GetBoolean("DB/Button 2", false);
		bool button4 = SmartDashboard::GetBoolean("DB/Button 3", false);

		while (IsAutonomous() && IsEnabled() && (button1 == true))
		{
			rightDriveEncoder.Reset();
			chassis.TankDrive(0,.3);
			distance = rightDriveEncoder.GetDistance();
			if (distance >= 19)
			{
				chassis.TankDrive(0.0,0.0);
			}


		}
		while (IsAutonomous() && IsEnabled() && (button2 == true))
		{
		}
		while (IsAutonomous() && IsEnabled() && (button3 == true))
		{

		}
		while (IsAutonomous() && IsEnabled() && (button4 == true))
		{

		}
	}

	void OperatorControl()
	{
		chassis.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
		{
			PotDeBouncePrint();												// Calling function PotDeBouncePrint to print Pot to Station.
			//H-Drive
			if (gamepad.GetRawButton(6) == true) //Should not be a nested while loop, you will stop execution of other items.
			{
				chassis.ArcadeDrive(gamepad.GetRawAxis(LEFT_STICK_Y_AXIS), gamepad.GetRawAxis(RIGHT_STICK_X_AXIS)*-1);
				hDriveMotor.Set(gamepad.GetRawAxis(LEFT_STICK_X_AXIS)*-1);
			}
			else{
				chassis.ArcadeDrive(gamepad.GetRawAxis(LEFT_STICK_Y_AXIS), gamepad.GetRawAxis(RIGHT_STICK_X_AXIS)*-1);
			}
			//PID
			if (manipulator.GetRawButton(5) == true){
				lowerStage.Set(PIDloop(LoadSetPoint));
			}

			//Two-Stage Combined Control Using x-axis of Logitech Joystick Attack 3
			if (manipulator.GetRawButton(5) != true && manipulator.GetRawButton(6) != true && manipulator.GetRawButton(7) != true &&
					manipulator.GetRawButton(8) != true && manipulator.GetRawButton(9) != true && manipulator.GetRawButton(10) != true)
			{
				// Allows for chain movement of the two lift stages in the up direction, limiting each when they reach their max height.
				if (lowerStagePot.GetVoltage() < lowerLiftMax && manipulator.GetRawAxis(1) < 0 )
				{
					lowerStage.Set(manipulator.GetRawAxis(1));
				}
				else if(lowerStagePot.GetVoltage() >= lowerLiftMax && manipulator.GetRawAxis(1) < 0)
				{
					lowerStage.Set(0);
					if (upperStagePot.GetVoltage() <= upperLiftMax && manipulator.GetRawAxis(1) < 0)
					{
						upperStage.Set(manipulator.GetRawAxis(1));
					}
					else
					{
						upperStage.Set(0);
					}
				}

				// This loop allows for the chain movement of the forklift in the downward direction, stopping it when in base position.
				if (upperStagePot.GetVoltage() > upperLiftMin && manipulator.GetRawAxis(1) > 0)
				{
					upperStage.Set(manipulator.GetRawAxis(1));
				}
				else if(upperStagePot.GetVoltage() <= upperLiftMin && manipulator.GetRawAxis(1) > 0)
				{
					upperStage.Set(0);
					if (lowerStagePot.GetVoltage() >= lowerLiftMin && manipulator.GetRawAxis(1) > 0)
					{
						lowerStage.Set(manipulator.GetRawAxis(1));
					}
					else
					{
						lowerStage.Set(0);
					}
				}
			}
		}
	}

	float  PIDloop(float SetPoint) //Currently only setup for Proportion
	{
		float MotorOut = 0;
		double Offset = SetPoint - lowerStagePot.GetVoltage(); // Difference between Pot and desired angle
		MotorOut = Proportion * Offset; //That difference is then multiplied by the P constant
		return MotorOut;
	}

	void PotDeBouncePrint()
	{
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
