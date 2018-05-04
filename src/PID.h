#ifndef PID_H
#define PID_H

#include <vector>

#ifndef LEAST_ERROR
#define LEAST_ERROR 0.001
#endif

#ifndef VAL_STEP
#define VAL_STEP 100
#endif

#ifndef TEST_STEP
#define TEST_STEP 200
#endif

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double best_error;
  double total_error;
  double error;
  int step;
  bool use_twiddle;

  /*
   * parameter, P, I, D
   */
  std::vector<double> P;
  std::vector<double> Dp;
  std::vector<double> best_p;

  // index of paramters
  int index;
  void IndexMove();

  int mark_num;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd, bool use_twiddle=true);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
