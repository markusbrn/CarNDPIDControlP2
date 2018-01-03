#include "PID.h"
#include <numeric>
#include <cmath>
#include <iostream>

using namespace std;

/*
* Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::UpdateError(double cte, double dt) {
	this->d_error = (cte - p_error) / dt;
	this->i_error += cte * dt;
	this->p_error = cte;
}

double PID::TotalError() {
	return (-Kp * p_error - Ki * i_error - Kd * d_error);
}

void PID::Twiddle(double cte, double dist_inc) {
	// initialize twiddle parameters
	if(!initialized) {
		dist = 0;
		dist_max = 2550;
		step = 0;
		cte_acc = 0.;
		cte_best = 1000.;
		run = true;
		i = 0;
		lap_count = 0;

		p = {Kp, Ki, Kd};
		dp = {0.01, 0.001, 0.01};
		initialized = true;
	}

	// run twiddle
	if(run == false) {
		cout << "cte_acc: " << cte_acc_eval << ", cte_best: " << cte_best << endl;
		if(step == 0) {
			p[i] += dp[i];
			this->Init(p[0], p[1], p[2]);
			cout << "Kp = " << p[0] << "Ki = " << p[1] << "Kd = " << p[2] << endl;
			step = 1;
			run = true;
		} else if(step == 1) {
			if(cte_acc_eval < cte_best) {
				cte_best = cte_acc_eval;
				dp[i] *= 1.1;
				step = 0;
				i = fmod(i+1,3);
			} else {
				p[i] -= 2*dp[i];
				this->Init(p[0], p[1], p[2]);
				cout << "Kp = " << p[0] << "Ki = " << p[1] << "Kd = " << p[2] << endl;
				step = 2;
				run = true;
			}
		} else if(step == 2) {
			if(cte_acc_eval < cte_best) {
				cte_best = cte_acc_eval;
				dp[i] *= 1.1;
			} else {
				p[i] += dp[i];
				this->Init(p[0], p[1], p[2]);
				cout << "Kp = " << p[0] << "Ki = " << p[1] << "Kd = " << p[2] << endl;
				dp[i] *= 0.9;
			}
			step = 0;
			i = fmod(i+1,3);
		}
	} else {
		cte_acc += pow(cte,2);
		dist += dist_inc;
		if(dist >= dist_max) {
			if(lap_count >= 1) {
				run = false;
			}
			cte_acc_eval = cte_acc;
			cte_acc = 0;
			dist = 0;
			lap_count += 1;
		}
	}
}

