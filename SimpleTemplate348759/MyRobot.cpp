		/* THIS FILE CONTAINS REMOVED LINES FROM MyRobot.cpp*/
		
		
		
		#include "SmartDashboard/SendableChooser.h"
		#include "SmartDashboard/Sendable.h"
		#include "SmartDashboard/NamedSendable.h"
		/*#include "Target.h"
		#include "BinaryImage.h"
		#include "VisionAPI.h"
		*/ 
		// #@$%#@$^%^@$%^!#$%#$^@$%&&$%$^!$#%#@$% http://www.chiefdelphi.com/forums/showthread.php?t=101624 is the forum page
		
		
		
		
		
		
		
		
		
					//Switch between inverted and non-inverted Joystick control.
			/*if(stick1->GetRawButton(8) && blnDriveCtrl == false)
			{
				//timerDriveCtrl.Reset();
				//timerDriveCtrl.Start();
				blnDriveCtrl = true;
				Wait(0.2);
			}
			else if(stick1->GetRawButton(8) && blnDriveCtrl == true)
			{
				blnDriveCtrl = false;
				Wait(0.2);
			}
			
			while(blnDriveCtrl == true)
			{
				singleDrive((stick1->GetY())*(-1.0),(stick1->GetX())*(-1.0),true);
				GetWatchdog().Feed();
				Wait(0.01);
			}
			
			while(blnDriveCtrl == false)
			{	
				singleDrive((stick1->GetY()),(stick1->GetX()),true);
				GetWatchdog().Feed();
				Wait(0.01);
			}
			*/
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
			if(stick2->GetRawButton(3) && blnFire == false)
			{
				//myShooter1.Set(-1);
				//myShooter2.Set(-1);
				myShooter1.Set((stick2->GetRawAxis(3))/2);
				myShooter2.Set((stick2->GetRawAxis(3))/2);
				SmartDashboard::PutString("Shooter","On");
				SmartDashboard::PutNumber("Shooter Speed (%)",fltShoot);
				blnFire = true;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
			}
			else if(stick2->GetRawButton(3) && blnFire == true)
			{
				myShooter1.Set(0);
				myShooter2.Set(0);
				blnFire = false;
				SmartDashboard::PutString("Shooter","Off");
				SmartDashboard::PutNumber("Shooter Speed (%)",0);
				GetWatchdog().Feed();
				Wait(0.5);
				
				GetWatchdog().Feed();
			}
			
			if(stick2->GetRawButton(2) && blnCtrlFire == false)
			{
				myShooter1.Set(-1);
				myShooter2.Set(-1);
				//myShooter1.Set(-(stick2->GetRawAxis(3))/2);
				//myShooter2.Set(-(stick2->GetRawAxis(3))/2);
				SmartDashboard::PutString("Shooter","On");
				SmartDashboard::PutNumber("Shooter Speed (%)",fltShoot);
				blnCtrlFire = true;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
			}
			else if(stick2->GetRawButton(2) && blnCtrlFire == true)
			{
				myShooter1.Set(0);
				myShooter2.Set(0);
				blnCtrlFire = false;
				SmartDashboard::PutString("Shooter","Off");
				SmartDashboard::PutNumber("Shooter Speed (%)",0);
				GetWatchdog().Feed();
				Wait(0.5);
				
				GetWatchdog().Feed();
			}
			
			if(stick2->GetRawButton(4) && blnFire90 == false)
			{
				myShooter1.Set(-0.9);
				myShooter2.Set(-0.9);
				SmartDashboard::PutString("Shooter","On");
				SmartDashboard::PutNumber("Shooter Speed (%)",90);
				blnFire90 = true;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
			}
			else if(stick2->GetRawButton(4) && blnFire90 == true)
			{
				myShooter1.Set(0);
				myShooter2.Set(0);
				blnFire90 = false;
				SmartDashboard::PutString("Shooter","Off");
				SmartDashboard::PutNumber("Shooter Speed (%)",0);
				GetWatchdog().Feed();
				Wait(0.5);
				
				GetWatchdog().Feed();
			}
			
			if(stick2->GetRawButton(5) && blnFire80 == false)
			{
				myShooter1.Set(-0.8);
				myShooter2.Set(-0.8);
				SmartDashboard::PutString("Shooter","On");
				SmartDashboard::PutNumber("Shooter Speed (%)",80);
				blnFire80 = true;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
			}
			else if(stick2->GetRawButton(5) && blnFire80 == true)
			{
				myShooter1.Set(0);
				myShooter2.Set(0);
				blnFire80 = false;
				SmartDashboard::PutString("Shooter","Off");
				SmartDashboard::PutNumber("Shooter Speed (%)",0);
				GetWatchdog().Feed();
				Wait(0.5);
				
				GetWatchdog().Feed();
			}
			
			if(stick2->GetRawButton(7) && blnFire70 == false)
			{
				myShooter1.Set(-0.7);
				myShooter2.Set(-0.7);
				SmartDashboard::PutString("Shooter","On");
				SmartDashboard::PutNumber("Shooter Speed (%)",70);
				blnFire70 = true;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
			}
			else if(stick2->GetRawButton(7) && blnFire70 == true)
			{
				myShooter1.Set(0);
				myShooter2.Set(0);
				blnFire70 = false;
				SmartDashboard::PutString("Shooter","Off");
				SmartDashboard::PutNumber("Shooter Speed (%)",0);
				GetWatchdog().Feed();
				Wait(0.5);
				
				GetWatchdog().Feed();
			}
				
			if(stick2->GetRawButton(6) && blnFire60 == false)
			{
				myShooter1.Set(-0.6);
				myShooter2.Set(-0.6);
				SmartDashboard::PutString("Shooter","On");
				SmartDashboard::PutNumber("Shooter Speed (%)",60);
				blnFire60 = true;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
			}
			else if(stick2->GetRawButton(6) && blnFire60 == true)
			{
				myShooter1.Set(0);
				myShooter2.Set(0);
				blnFire60 = false;
				SmartDashboard::PutString("Shooter","Off");
				SmartDashboard::PutNumber("Shooter Speed (%)",0);
				GetWatchdog().Feed();
				Wait(0.5);
				
				GetWatchdog().Feed();
			}
