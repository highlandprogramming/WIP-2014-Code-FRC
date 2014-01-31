#include "WPILib.h"
#include "DashboardDataFormat.h" 
//#include "NamedSendable.h"
//#include "Sendable.h"
//#include "SendableChooser.h"
//#include "SmartDashboard.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	DashboardDataFormat dashboardDataFormat;
	RobotDrive myRobot; // robot drive system
	Joystick *stick1; 
	Joystick *stick2; 
	Solenoid *s[8];

public:

	RobotDemo():
		myRobot(1,2,3,4)// these must be initialized in the same order
	
	{
		GetWatchdog().SetExpiration(0.1);
		stick1 = new Joystick(1);
		stick2 = new Joystick(2);
		//compressor = new Compressor(1,1);
		s[0] = new Solenoid(1);
		s[1] = new Solenoid(2);
		s[2] = new Solenoid(3);
		s[3] = new Solenoid(4);
		s[4] = new Solenoid(5);
		s[5] = new Solenoid(6);
		s[6] = new Solenoid(7);
		s[7] = new Solenoid(8); 
		myRobot.SetExpiration(0.1);
	}
	//decostructor  
~RobotDemo()
{
	delete s[7];
	delete s[6];
	delete s[5];
	delete s[4];
	delete s[3];
	delete s[2];
	delete s[1];
	delete s[0];
	//delete compressor;
	delete stick1;
	delete stick2;
	
}
	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous()
	{
		GetWatchdog().SetEnabled(false);
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl()
	{
		GetWatchdog().SetEnabled(false);
				
				while (true)
				{
					GetWatchdog().Feed();
					dashboardDataFormat.SendIOPortData();
					dashboardDataFormat.SendVisionData();
					Wait(1.0);
				}
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick1); // drive with arcade style (use right stick)
					// wait for a motor update time
		}
	}
	
	/**
	 * Runs during test mode
	 */
	};

START_ROBOT_CLASS(RobotDemo);

