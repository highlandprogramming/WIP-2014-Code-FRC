#include "OI.h"

OI::OI() {
	// Process operator interface input here.
	joystick1 = new Joystick(1);
	
	button1 = new JoystickButton(joystick1, 1);
	
	
	
}
