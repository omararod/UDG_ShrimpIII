#include "UDG_ShrimpIII.h"
#include <unistd.h>
int main()
{
	UDG_ShrimpIII robot("/dev/ttyUSB0");
	robot.TurnRobotOn();
	robot.SetVelocityAndSteeringAngle(0,0);
	sleep(2);
	robot.TurnRobotOff();
}
