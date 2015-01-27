#include "WPILib.h"
#include "math.h"
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
	AnalogInput pot; 	  			// Potentiometer for positioning of eveutual forklift.
	Solenoid hDrive;				// Pneumatic cilender for deploying H drive system.
	Talon hDriveMotor;				// Motor running on H drive system.
	PowerDistributionPanel pdBoard;	// pdBoard used for hardware debug
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
			ultraSonic (0), 		// Sensor feeding data into Analog input port 0.
			pot (1), 				// Sensor feeding data into Analog input port 1.
			hDrive(0),				// Solenoid for hDrive is on port 0 of the pneumatic controller.
			hDriveMotor(4)			// H drive motor.
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
			chassis.ArcadeDrive(gamepad.GetRawAxis(1), gamepad.GetRawAxis(LEFT_STICK_X_AXIS)*-1);
			hDriveMotor.Set(gamepad.GetRawAxis(RIGHT_STICK_X_AXIS)*-1);
			Wait(0.0005);
			/*while(gamepad.GetRawButton(6) == true)							// While I'm pressing button 6.
			{
				hDrive.Set(true);											// Engage actuator contacting h drive wheel with ground.
				hDriveMotor.Set(gamepad.GetRawAxis(2)*-1);						// Run H drive motor referencing x movement on right stick.
			}
			*/
			distanceVoltage = ultraSonic.GetVoltage(); 						// Reading raw voltage from ultra sonic into variable distanceVoltage.
			oldAngle = angle;		  										// Take the last angle and store it as the old angle.
			angle = pot.GetVoltage(); 										// Read voltage in from pot and store into variable angle.
			distance = distanceVoltage / 0.00977;	   						// Converting voltage to inches with scaler.
			roundedDistance = ceilf(distance * 100) / 100; 					// Doing calculations to round distance value.
			SmartDashboard::PutNumber("Distance", roundedDistance); 		// Print the distance to SmartDashboard (not so smart).
			upperLim = oldAngle + 0.01;										// Set an upper limit to compare new angle to
			lowerLim = oldAngle - 0.01;										// Set a lower limit to compare new angle to
			if ((angle > upperLim) || (angle < lowerLim))					// If angle has actually changed a considerable amount...
			{
				SmartDashboard::PutNumber("Angle", angle);  				// Print it out to the dashboard.
			}
			if ((angle < upperLim) || (angle > lowerLim))
			{
				Wait(.001);
			}
		}

	}

	void Test()
	{
#if 1
#define PORT_COUNT 16
		double ports[PORT_COUNT];
		for (int i = 0; i < PORT_COUNT; i++)
			ports[i] = pdBoard.GetCurrent(i);
		for (int i = 0; i < PORT_COUNT; i++) {
			char msg[64];
			sprintf(msg, "Current Draw %d: ", i);
			SmartDashboard::PutNumber(msg, ports[i]);	// Print that value out to the Smart Dasboard for Debug
		}
#else
		double port0 = pdBoard.GetCurrent(0);	//Read a particular current draw from a port and store it into a variable.
		double port1 = pdBoard.GetCurrent(1);
		double port2 = pdBoard.GetCurrent(2);
		double port3 = pdBoard.GetCurrent(3);
		double port4 = pdBoard.GetCurrent(4);
		double port5 = pdBoard.GetCurrent(5);
		double port6 = pdBoard.GetCurrent(6);
		double port7 = pdBoard.GetCurrent(7);
		double port8 = pdBoard.GetCurrent(8);
		double port9 = pdBoard.GetCurrent(9);
		double port10 = pdBoard.GetCurrent(10);
		double port11 = pdBoard.GetCurrent(11);
		double port12 = pdBoard.GetCurrent(12);
		double port13 = pdBoard.GetCurrent(13);
		double port14 = pdBoard.GetCurrent(14);
		double port15 = pdBoard.GetCurrent(15);

		SmartDashboard::PutNumber("Current Draw 0: ", port0);	// Print that value out to the Smart Dasboard for Debug
		SmartDashboard::PutNumber("Current Draw 1: ", port1);
		SmartDashboard::PutNumber("Current Draw 2: ", port2);
		SmartDashboard::PutNumber("Current Draw 3: ", port3);
		SmartDashboard::PutNumber("Current Draw 4: ", port4);
		SmartDashboard::PutNumber("Current Draw 5: ", port5);
		SmartDashboard::PutNumber("Current Draw 6: ", port6);
		SmartDashboard::PutNumber("Current Draw 7: ", port7);
		SmartDashboard::PutNumber("Current Draw 8: ", port8);
		SmartDashboard::PutNumber("Current Draw 9: ", port9);
		SmartDashboard::PutNumber("Current Draw 10: ", port10);
		SmartDashboard::PutNumber("Current Draw 11: ", port11);
		SmartDashboard::PutNumber("Current Draw 12: ", port12);
		SmartDashboard::PutNumber("Current Draw 13: ", port13);
		SmartDashboard::PutNumber("Current Draw 14: ", port14);
		SmartDashboard::PutNumber("Current Draw 15: ", port15);
#endif
		}

};

START_ROBOT_CLASS(Robot);
