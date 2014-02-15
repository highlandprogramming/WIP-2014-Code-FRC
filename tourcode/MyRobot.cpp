#include "WPILib.h"
#include "Solenoid.h"
#include "Compressor.h"
#include "Victor.h"

class RobotDemo : public SimpleRobot
{
	
	RobotDrive myRobot;// robot drive system
	Victor* gravity; //pulls catapolt down //wheel1
	Victor* commas;//pull the pin to launch the grenade//wheel2
	Joystick stick1; 
	Joystick stick2;
	Compressor *compressor;
	Solenoid *s[8];
	bool blnshift;
	bool hooks;
	int firepin;
	int ludicrous;
	
	 
public:
	RobotDemo():
		
			
		myRobot(1,2),// these must be initialized in the same order as they are declared above.
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
				commas = new Victor(9),
				myRobot.SetExpiration(0.1);
	}
		//Deconstructor
			~RobotDemo()
			{
				delete commas;
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
		
		//Pulldown.SetSafetyEnabled(true);
		ludicrous = 1; 
		firepin = 1;		
		compressor->Start();
		myRobot.SetSafetyEnabled(true);
		hooks =false;
		
		while (IsOperatorControl())
		{
			//Pulldown.ArcadeDrive(stick2);
			//gravity->SetSpeed(0.5);
			//grenade->SetSpeed(0.5);
			
			GetWatchdog().Feed();
			myRobot.ArcadeDrive(stick1); // drive with arcade style (use right stick)
			Wait(0.005);// wait for a motor update time
			
	//-----------------------------------------------------------------
			if(stick2.GetRawButton(1) == true)//firing wheels
			{
				GetWatchdog().Feed();
				myRobot.ArcadeDrive(stick1); 
				gravity->SetSpeed(-0.8);
				commas->SetSpeed(-0.8);
				if(stick2.GetRawButton(3)== true)// firing piston out
							{
								GetWatchdog().Feed();
								myRobot.ArcadeDrive(stick1); 
								s[1]->Set(true);
								
							}
							else if(stick2.GetRawButton(2)== true)// firing piston in
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
				commas->SetSpeed(0.0);
			}
	//----------------------------------------------------------------
			if(stick2.GetRawButton(3)== true)// firing piston out
			{
				GetWatchdog().Feed();
				myRobot.ArcadeDrive(stick1); 
				s[1]->Set(true);
				
			}
			else if(stick2.GetRawButton(2)== true)// firing piston in
			{
				GetWatchdog().Feed();
				myRobot.ArcadeDrive(stick1); 
				s[1]->Set(true);
			}
	//---------------------------------------------------------------------
			if(stick2.GetRawButton(9)==(true))//hooks up
					{
						GetWatchdog().Feed();
						myRobot.ArcadeDrive(stick1); 
						hooks = true;
						Wait(0.005);
					}
				else if(stick2.GetRawButton(8)== (true))
					{
						GetWatchdog().Feed();
						myRobot.ArcadeDrive(stick1); 
						hooks = false;
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
			
			
			/*if(stick1.GetRawButton(7))//SUPER SHIFTERS low gear
				{
					GetWatchdog().Feed();
					myRobot.ArcadeDrive(stick1);
					s[3]->Set(true);
				
				}
			else if(stick1.GetRawButton(6))//SUPERSTIFTERS high gear
				{
					GetWatchdog().Feed();
					myRobot.ArcadeDrive(stick1);
					s[3]->Set(false);
				}*/
			
			
		
			/*if(stick2.GetRawButton(5) == (true))//fireing pin on trigger
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



