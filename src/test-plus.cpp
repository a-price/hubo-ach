#include "hubo_plus.h"



int main(int argc, char **argv)
{
	
	hubo_plus hubo;

/*	Eigen::VectorXd angles(6);
	angles << -2, -0.3, -2.25, -2.0, -2.0, -1.25;


	hp_flag_t result = hubo.setLeftArmAngles(angles);

	Eigen::VectorXd vels(6);
	hubo.getLeftArmNomSpeeds(vels);

	printf("%f\t%f\t%f\n", vels[0], vels[1], vels[2]);*/

	hubo.setJointAngle(LSP,1);

	hubo.sendControls();

}





