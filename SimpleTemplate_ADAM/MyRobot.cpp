#include "WPILib.h"
#include "Solenoid.h"
#include "Compressor.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	//RobotDrive Pulldown;
	
	RobotDrive myRobot;// robot drive system
	Victor gravity; //pulls catapolt down 
	Victor grenade;//pull the pin to launch the grenade
	Joystick stick1; 
	Joystick stick2;
	Compressor *compressor;
	Solenoid *s[8];
	bool blnshift;
	int firepin;
	int ludicrous;
	//JoystickButton fire;
	 
public:
	RobotDemo():
		
		//Pulldown(3),	// these must be initialized in the same order
		myRobot(1, 2),	// as they are declared above.
		gravity(3),
		grenade(4),  
		stick1(1),
		stick2(2)
		
	//	s[0] = new Solenoid(1)
		//this->pSolenoid = new Solenoid
		//fire(stick2(1))
		
	{
		//Init-----------
				compressor = new Compressor(1,1);
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
		//Deconstructor
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
				delete compressor;
				
				
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
		ludicrous = 1; 
		firepin = 1;		
		compressor->Start();
		myRobot.SetSafetyEnabled(true);
		
		while (IsOperatorControl())
		{
			
			//Pulldown.ArcadeDrive(stick2);
			myRobot.ArcadeDrive(stick1); // drive with arcade style (use right stick)
			Wait(0.005);// wait for a motor update time
			
			if(stick1.GetRawButton(7))//SUPER SHIFTERS 
			{
				if(ludicrous == 1)
				{
					s[1].Set(true);
					ludicrous =2;
				}
				else if(ludicrous == 3)
				{
					s[1].Set(false);
					ludicrous = 4;
				}
			}
			else
			{
				if(ludicrous == 2)
				{
					ludicrous = 3;
				}
				else if(ludicrous==4)
				{
					ludicrous = 1; 
				}
			}
		
			if(stick2.GetRawButton(1) == (true))//fireing pin (pulls back and fires)
			{
				GetWatchdog().Feed();
				if(firepin == 1)// pull back
				{
					GetWatchdog().Feed();
					myRobot.ArcadeDrive(stick1); 
					s[0]->Set(true);
					gravity.SetSpeed(0.5);
					Wait(1.0);					
					firepin = 2;
					
					
				}
				else if(firepin == 3)//shift to N and fire
				{
					GetWatchdog().Feed();
					s[0]->Set(false);
					grenade.SetSpeed(1.0);
					Wait(0.005);
					firepin = 4;
				}
			}
			else
			{
				if(firepin == 2)
				{
					GetWatchdog().Feed();
					firepin = 3;
				}
				else if(firepin == 4)
				{
					GetWatchdog().Feed();
					firepin = 1;
				}
			}
			
		}
	}
	/*
	 * Runs during test mode
	 */
	void Test()
	{

	}
};

START_ROBOT_CLASS(RobotDemo);

