#include <iostream>
#include <string>
#include <uWS/uUV.h>
#include <math.h>
#include "PID.h"
#include <vector>

using namespace std;

PID::PID(){}

PID::~PID(){}

void PID::Init(double Kp,double Ki,double Kd){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

void PID::UpdateError(double cte){
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError(){
    // Keeping track of all previous CTEs.
    // initially we set the int_cte to 0 then add the current cte to the count int_cet +=cte.
    // Finally we update the steering value
    //steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
    return -Kp * p_error - Kd *d_error - Ki*i_error;
}

void PID::Twiddle(double cte,PID p_main){
    //std::vector<double> p_local = {0.06,0.0005,3.0};
    std::vector<double> p_local = {p_main.Kp,p_main.Ki,p_main.Kd};
    std::vector<double> dp_local = {1.,1.,1.};
    std::cout<< "PID Starting Point: ["<< p_local[0]<<"]["<<p_local[1]<<"]["<<p_local[2]<<"]\n"<<std::endl;
    
    static double best_err = 0.0;
    static bool twiddleInit = false;
    if (!twiddleInit){
        std::cout<<"Twiddle init"<<std::endl;
        best_err = cte;
        twiddleInit=true;
        return;
    }
    //double best_err = cte; //TotalError();
    std::cout<< "==========================Best Error at starting point:["<< best_err<<"]\n"<<std::endl;

    double tol_=0.2;
    int icounter=0;

    while ((dp_local[0]+dp_local[1]+dp_local[2])>tol_){
        //std::cout<< "sum of dp_local: "<< dp_local[0]+dp_local[1]+dp_local[2] << " : iteration counter:" <<icounter<<", best error:"<<best_err<<std::endl;
        for (int j=0;j<p_local.size();j++){
            p_local[j] += dp_local[j];
            // this->Kp = p_local[0];
            // this->Ki = p_local[1];
            // this->Kd = p_local[2];

            std::cout<<"Entry of for loop Kp:[" << Kp << "] Ki:[" << Ki << "] Kd:[" << Kd <<"]" <<std::endl;
            //std::cout<<"p_error:"<<p_error<<" i_error:"<<i_error<<" d_error:"<<d_error<<std::endl;
            double err = TotalError();
            //std::cout<<"err after TotalError():"<<err<<" value of k:"<<k<<std::endl;
            if (abs(err) < abs(best_err)) {
                best_err = abs(err);
                dp_local[j] *= 1.1;
                std::cout<<"Increasing the value at 1st attempt"<<std::endl;
                std::cout<< "Best Error at above point: ["<< best_err<<"]\n"<<std::endl;
            }
            else {
                p_local[j] -= 2*dp_local[j];
                //std::cout<<"1st print"<<p_local[j]<<"::"<<dp_local[j]<<std::endl;
                this->Kp = p_local[0];
                this->Ki = p_local[1];
                this->Kd = p_local[2];
                //std::cout<<"p_local[k] -= 2*dp_local[k]:"<<"Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd << std::endl;
                //std::cout<<"p_error:"<<p_error<<" i_error:"<<i_error<<" d_error:"<<d_error<<std::endl;
                err = TotalError();
                //std::cout<<"err after TotalError():"<<err<<"value of i:"<<k<<std::endl;
                if (abs(err) < abs(best_err)) {
                    best_err = abs(err);
                    dp_local[j] *= 1.1;
                    std::cout<<"Increasing the value after 2nd attempt."<<std::endl;
                    std::cout<< "Best Error at above point: "<< best_err<<"\n"<<std::endl;
                } else {
                    p_local[j] += dp_local[j];
                    dp_local[j] *= 0.9;
                    //std::cout<<p_local[j]<<"::"<<dp_local[j]<<std::endl;
                    std::cout<<"lowering the value..."<<std::endl;
                    std::cout<< "Best Error at above point: "<< best_err<<"\n"<<std::endl;
                    //std::cout<<"p_error:"<<p_error<<" i_error:"<<i_error<<" d_error:"<<d_error<<std::endl;
                } // inner  if (abs(err) < best_err)
            } // if (abs(err) < best_err)
            std::cout<<"At bottom of loop Kp:[" << Kp << "] Ki:[" << Ki << "] Kd:[" << Kd <<"]"<<std::endl;
            std::cout<<"p_local(p):[" << p_local[0] << "] p_local(i):[" << p_local[1] << "] p_local(d):[" << p_local[2] <<"]\n"<<std::endl;
            std::cout<<"dp_local(p):[" << dp_local[0] << "] dp_local(i):[" << dp_local[1] << "] dp_local(d):[" << dp_local[2] <<"]\n"<<std::endl;
            std::cout<<"===============Moving to next for PID=======\n"<<std::endl;
        } // for loop
        std::cout<<"===========================================Moving to iteration for tol_=======\n\n"<<icounter<<std::endl;
        icounter +=1;
        //if (icounter>2){break;}
    } //while loop
    std::cout<< "=================================================================Best Error at after tolerance value is met: ["<< best_err<<"]\n"<<std::endl;
} // Twiddle end
