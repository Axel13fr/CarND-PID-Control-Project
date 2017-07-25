#ifndef PID_H
#define PID_H

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
  double Kp_;
  double Ki_;
  double Kd_;

  double diff_cte;
  double prev_cte;
  double int_cte;

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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /**
   * @brief ControlSteering
   * @param angle the currently applied angle
   * @return
   */
  double ControlSteering(double cte);
};

#endif /* PID_H */
