#ifndef PID_H
#define PID_H

#include <vector>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle Parameters
  */
  double dist;
  double dist_max;
  int step;
  double cte_acc;
  double cte_acc_eval;
  double cte_best;
  bool run;
  int i;
  int lap_count;
  vector<double> p;
  vector<double> dp;
  bool initialized;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double dt);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle(double cte, double dist_inc);
};

#endif /* PID_H */
