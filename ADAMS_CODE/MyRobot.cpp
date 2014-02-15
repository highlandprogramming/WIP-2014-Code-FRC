#include "WPILib.h"
#include "Solenoid.h"
#include "Compressor.h"
#include "Victor.h"

class RobotDemo : public SimpleRobot
{
	
	RobotDrive myRobot;// robot drive system
	Victor* gravity; //pulls catapolt down //wheel1
	Victor* grenade;//pull the pin to launch the grenade//wheel2
	Joystick stick1; 
	Joystick stick2;
	Compressor *compressor;
	Solenoid *s[8];
	bool blnShift;
	//bool hooks;
	int firepin;
	int ludicrous;
	Timer supershiftertimer;
	
	 
public:
	RobotDemo():
		
			
		myRobot(1,3,2,4),// these must be initialized in the same order as they are declared above.
		//grenade(6),  
		stick1(1),
		stick2(2)
		
	
		
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
				gravity = new Victor(8),
				grenade = new Victor(9),
				myRobot.SetExpiration(0.1);
	}
		//Deconstructor
			~RobotDemo()
			{
				delete grenade;
				delete gravity;
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
		supershiftertimer.Reset();
		supershiftertimer.Start();
		blnShift = false;
		//Pulldown.SetSafetyEnabled(true);
		ludicrous = 1; 
		firepin = 1;		
		compressor->Start();
		myRobot.SetSafetyEnabled(true);
	//	hooks =false;
		
		while (IsOperatorControl())
		{
			//Pulldown.ArcadeDrive(1);
			//gravity->SetSpeed(0.5);````````````````
			//grenade->SetSpeed(0.5);
			
			GetWatchdog().Feed();
			myRobot.ArcadeDrive(stick1); // drive with arcade style (use right stick)
			
	//-----------------------------------------------------------------
		/*	if(stick1.GetRawButton(1) == true)//firing wheels
			{
				GetWatchdog().Feed();
				myRobot.ArcadeDrive(stick1); 
				gravity->SetSpeed(-0.8);
				grenade->SetSpeed(-0.8);
				if(stick1.GetRawButton(3)== true)// firing piston out
							{
								GetWatchdog().Feed();
								myRobot.ArcadeDrive(stick1); 
								s[1]->Set(true);
								
							}
							else if(stick1.GetRawButton(2)== true)// firing piston in
							{
								GetWatchdog().Feed();
								myRobot.ArcadeDrive(stick1); 
								s[1]->Set(true);
							}
				
			}
			else
			{
				GetWatchdog().Feed();
				myRobot.ArcadeDrive(stick1);
				gravity->SetSpeed(0.0);
				grenade->SetSpeed(0.0);
			}
	//----------------------------------------------------------------
			if(stick1.GetRawButton(3)== true)// firing piston out
			{
				GetWatchdog().Feed();
				myRobot.ArcadeDrive(stick1); 
				s[1]->Set(true);
				
			}
			else if(stick1.GetRawButton(2)== true)// firing piston in
			{
				GetWatchdog().Feed();
				myRobot.ArcadeDrive(stick1); 
				s[1]->Set(true);
			}
	//---------------------------------------------------------------------
			if(stick1.GetRawButton(9)==(true))//hooks up
					{
						GetWatchdog().Feed();
						myRobot.ArcadeDrive(stick1); 
						//hooks = true;
						Wait(0.005);
					}
				else if(stick1.GetRawButton(8)== (true))
					{
						GetWatchdog().Feed();
						myRobot.ArcadeDrive(stick1); 
						//hooks = false;
						Wait(0.005);
					}
					
				if(hooks == true)
					{
						GetWatchdog().Feed();
						myRobot.ArcadeDrive(stick1); 
						
						s[3]->Set(true);
				
					}
				else if(hooks == false)
					{
						GetWatchdog().Feed();
						myRobot.ArcadeDrive(stick1);
						s[3]->Set(false);
					}
					*/
			
			if(supershiftertimer.Get() > 0.2)
			{
				if(stick1.GetRawButton(7)== true)//SUPER SHIFTERS low gear
					{
							GetWatchdog().Feed();
							s[1]->Set(true);
							supershiftertimer.Stop();
							supershiftertimer.Reset();
							supershiftertimer.Start();
							
					}
				else if(stick1.GetRawButton(6)== true)//SUPERSTIFTERS high gear
					{
							GetWatchdog().Feed();
							s[1]->Set(false);
							supershiftertimer.Stop();
							supershiftertimer.Reset();
							supershiftertimer.Start();
					}
				//if(GetRawButton(7)== true)
				//{
				//	s[7].Set(true);
				//}
			}
			
		
			/*if(stick.GetRawButton(5) == (true))//fireing pin on trigger
				{
					GetWatchdog().Feed();
					s[8]->Set(true);//SS N
					//s[1]->Set(true);
					Wait(1.0);
					s[8]->Set(false);
					GetWatchdog().Feed();
					
				}
			else
				{
					GetWatchdog().Feed();
					myRobot.ArcadeDrive(stick1);
				}*/
	
		/*	if(stick1.GetRawButton(2))// pullback on button 3
				{
					GetWatchdog().Feed();
					myRobot.ArcadeDrive(stick1);
					//s[8]->Set(true);
					//Wait(0.5);
					gravity->SetSpeed(0.5);
					grenade->SetSpeed(0.5);
					GetWatchdog().Feed();
					//Wait(1.0);					
				}
			else
				{
					GetWatchdog().Feed();
					myRobot.ArcadeDrive(stick1);
				}
				*/
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

