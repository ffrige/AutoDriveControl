#include "PID.h"
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	
	gs = 1.0;
}

void PID::UpdateError(double cte) {

	double error_in = (0 - cte);	//error = set_value - act_value. Our set value is zero.

	i_error += error_in;
	d_error = error_in - p_error; //p_error still holds the old error from before!
	p_error = error_in;

	//integral anti-windup
	if (i_error > 0.5) i_error = 0.5;
	if (i_error < -0.5) i_error = -0.5;
	
}

double PID::TotalError() {
	return (Kp * p_error * gs + Ki * i_error + Kd * d_error);
}

