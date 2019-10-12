/*
 * kalman.h
 *
 *  Created on: 2016Äê9ÔÂ7ÈÕ
 *      Author: matthew
 */

#ifndef KALMAN_H_
#define KALMAN_H_


class Kalman{
public:
	Kalman();
	float getAngle(float X_angle,float Gyro );
private:
	float Q_angle;
	float Q_gyro;
	float R_measure;

	float dt;
	float angle;
	float Q_bias;
	float PP[2][2];
};


#endif /* KALMAN_H_ */
