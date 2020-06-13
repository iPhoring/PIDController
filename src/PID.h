#ifndef PID_H
#define PID_H

class PID{
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
   * Twiddle
   */
   double best_err;

   /*
   * Constructor
   */
   PID();

   /*
   * Destructor
   */
   virtual ~PID();

   /*
   * Initialize PID
   */
   void Init(double Kp,double Ki,double Kd);

   /*
   * Update the PID error variables given cross track error.
   */
   void UpdateError(double cte);
   /*
   * Twiddle or vanilla gradient descent for tuning one hyper parameter as a time.
   */
   void Twiddle(double cte,PID p);

   /*
   * Calculate the total OID error.
   */
   double TotalError();
   
};

#endif /* PID_H */