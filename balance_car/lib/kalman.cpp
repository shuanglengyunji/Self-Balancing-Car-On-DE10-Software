
#include "kalman.h"


Kalman::Kalman()
{
	 Q_angle=0.001;
	 Q_gyro=0.003;
	 R_measure=0.5;

	 dt=0.01;
	 angle=0;
	 Q_bias=0;
	// PP[2][2]={{1.0,0},{0,1.0}};
	 PP[0][0]=1;
	 PP[0][1]=0;
	 PP[1][0]=0;
	 PP[1][1]=1;
}


float Kalman::getAngle(float X_angle,float Gyro )
{
	float Pdot[4] ={0,0,0,0};
	float  PCt_0,PCt_1;
	char   C_0 = 1;
	float  Angle_err;
	float  t_0,t_1;
	float  K_0,K_1;
	float  E;
	angle+=(Gyro-Q_bias)*dt;
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0];
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = X_angle - angle;

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_measure + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle	+= K_0 * Angle_err;
	Q_bias	+= K_1 * Angle_err;

	return angle;

}
