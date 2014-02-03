#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	//RobotDrive Pulldown;
	Victor gravity; //pulls catapolt down 	
	RobotDrive myRobot; // robot drive system
	Joystick stick1; 
	Joystick stick2;
	//JoystickButton fire;
	 
public:
	RobotDemo():
		//Pulldown(3),
		gravity(3),// these must be initialized in the same order
		myRobot(1, 2),	// as they are declared above.
		stick1(1),
		stick2(2)
		//fire(stick2(1))
		
	{
		myRobot.SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous()
	{
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
		//Pulldown.SetSafetyEnabled(true);
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			//Pulldown.ArcadeDrive(stick2);
			myRobot.ArcadeDrive(stick1); // drive with arcade style (use right stick)
			Wait(0.005);	// wait for a motor update time
		
			if(stick2.GetRawButton(1))
			{
				//fire piston out
			}
			else
			{
				//bring piston in 
			}
			
		}
	}
			/*if(stick1>GetRawButton(8)== true)
			{
				
				gravity.(0.5,0.0);
				Wait(3.0);
				gravity.(0.0,0.0); 
			}
			if(stick1>GetRawButton(8) == false)
						{
							
						}
		}
	}
	
	
	 * Runs during test mode
	 */
	void Test()
	{

	}
};

START_ROBOT_CLASS(RobotDemo);


