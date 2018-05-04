#include "PID.h"
#include "json.hpp"
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool use_twiddle) {
  P.push_back(Kp);
  P.push_back(Ki);
  P.push_back(Kd);
  step = 1;

  p_error = 0;
  d_error = 0;
  i_error = 0;

  best_error = std::numeric_limits<double>::max();
  total_error = 0;
  error = 0;
  mark_num = 0;
  index = 0;

  this->use_twiddle = use_twiddle;
  for (auto p : P) {
    if (p == 0) {
      Dp.push_back(0.1);
    } else {
      Dp.push_back(0.1 * p);
    }
  }
//  Dp = {0.0198, 0.000324, 0.243};
}

void PID::UpdateError(double cte) {
  if(step == 1){
    p_error = cte;

  }
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if(use_twiddle){
    if(step % (VAL_STEP + TEST_STEP) > VAL_STEP){
      total_error += cte * cte;
    }

    if(step % (VAL_STEP + TEST_STEP) == 0){
      std::cout<<"==============  step "<<step<<" =============="<<std::endl;
      std::cout << "P: "<< P[0]<<" I: "<<P[1]<<" D: "<<P[2]<<std::endl;
      std::cout << "Dp: [" << Dp[0] << ", " << Dp[1] << ", " << Dp[2] << "]" << std::endl;
      std::cout << "The best error: " << best_error << std::endl;
      if (step == VAL_STEP + TEST_STEP) {
        best_error = total_error / TEST_STEP;
        best_p.insert(best_p.end(), P.begin(), P.end());
      } else {
        error = total_error / TEST_STEP;
        std::cout << "The total error: " << total_error << " The error: " << error << std::endl;
        if (best_error > error) {
          Dp[index] *= 1.1;
          best_error = error;
          best_p.clear();
          best_p.insert(best_p.end(), P.begin(), P.end());
          this->IndexMove();
          P[index] += Dp[index];
          mark_num = 0;
        } else {
          if (mark_num == 0) {
            P[index] -= 2 * Dp[index];
            mark_num++;
          } else {
            P[index] += Dp[index];
            Dp[index] *= 0.9;
            this->IndexMove();
            P[index] += Dp[index];
            mark_num = 0;
          }
        }
      }
      total_error = 0;
      double sum_dp = 0;
      for (auto &dp : Dp) {
        sum_dp += dp;
      }
      std::cout << "sum_dp: " << sum_dp << std::endl;
      if (sum_dp < LEAST_ERROR) {
        use_twiddle = false;
        std::cout<<"==============  Last step "<<step<<" =============="<<std::endl;
        std::cout << "P: "<< best_p[0]<<" I: "<<best_p[1]<<" D: "<<best_p[2]<<std::endl;
        std::cout << "Dp: [" << Dp[0] << ", " << Dp[1] << ", " << Dp[2] << "]" << std::endl;
        std::cout << "The best error: " << best_error << std::endl;
      }
    }
  }
  step++;
}

double PID::TotalError() {
  return -P[0] * p_error - P[1] * i_error - P[2] * d_error;
}

void PID::IndexMove() {
  index++;
  if (index >= 3) {
    index = 0;
  }
}

