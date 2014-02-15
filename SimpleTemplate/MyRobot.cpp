#include "WPILib.h"
#include "Talon.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick stick1; // only joystick
	Joystick stick2;
	Compressor AC;
	Talon* gravity;
	Talon* grenade;
	Relay* relay1;

public:
	RobotDemo():
		myRobot(3,1,4,2),	// these must be initialized in the same order	
		stick1(1),
		stick2(2),
		AC (2,2)
		//relay1(2)
	
		
	{
		
		relay1 = new Relay(2);
		//relay1 =new Relay(2,2);
		relay1 = new Relay(2,Relay::kForwardOnly);
		
		//stick1 = new Joystick(1);// as they are declared above.
		//stick2 = new Joystick(2);
		//compressor = new Compressor(1,1);
		//gravity = new Talon(2),
		//grenade = new Talon(4),
		myRobot.SetExpiration(0.1);
	}

	~RobotDemo()
				{
					delete relay1;
					//delete grenade;
					//delete gravity;
					//delete compressor;
				}
	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous()
	{
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(-0.5, 0.5); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 * 
	 */
	void OperatorControl()
	{
		GetWatchdog().Feed(); 
		myRobot.SetSafetyEnabled(true);
		relay1->Set(Relay::kForward);
		AC.Start();
		//AC.Set(Relay::kForward);
		while (IsOperatorControl())
		{
			GetWatchdog().Feed(); 
			myRobot.ArcadeDrive(stick1); // drive with arcade style (use right stick)
			//Wait(0.005);				// wait for a motor update time
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);
