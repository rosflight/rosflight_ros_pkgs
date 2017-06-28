/*
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "fixedwing_forces_and_moments.h"

Fixedwing::Fixedwing(ros::NodeHandle* nh)
{
    nh_ = nh;

    // physical parameters
    mass_ = nh_->param<double>("mass", 13.5);
    Jx_ = nh_->param<double>("Jx", 0.8244);
    Jy_ = nh_->param<double>("Jy", 1.135);
    Jz_ = nh_->param<double>("Jz", 1.759);
    Jxz_ = nh_->param<double>("Jxz", .1204);
    rho_ = nh_->param<double>("rho", 1.2682);

    // Wing Geometry
    wing_.S = nh_->param<double>("wing_s", 0.55);
    wing_.b = nh_->param<double>("wing_b", 2.8956);
    wing_.c = nh_->param<double>("wing_c", 0.18994);
    wing_.M = nh_->param<double>("wing_M", 0.55);
    wing_.epsilon = nh_->param<double>("wing_epsilon", 2.8956);
    wing_.alpha0 = nh_->param<double>("wing_alpha0", 0.18994);

    // Propeller Coefficients
    prop_.k_motor = nh_->param<double>("k_motor", 80.0);
    prop_.k_T_P = nh_->param<double>("k_T_P", 0.0);
    prop_.k_Omega = nh_->param<double>("k_Omega", 0.0);
    prop_.e = nh_->param<double>("prop_e", 0.9);
    prop_.S = nh_->param<double>("prop_S", 0.202);
    prop_.C = nh_->param<double>("prop_C", 1.0);

    // Lift Params
    CL_.O = nh_->param<double>("C_L_O", 0.28);
    CL_.alpha = nh_->param<double>("C_L_alpha", 3.45);
    CL_.beta = nh_->param<double>("C_L_beta", 0.0);
    CL_.p = nh_->param<double>("C_L_p", 0.0);
    CL_.q = nh_->param<double>("C_L_q", 0.0);
    CL_.r = nh_->param<double>("C_L_r", 0.0);
    CL_.delta_a = nh_->param<double>("C_L_delta_a", 0.0);
    CL_.delta_e = nh_->param<double>("C_L_delta_e", -0.36);
    CL_.delta_r = nh_->param<double>("C_L_delta_r", 0.0);

    // Drag Params
    CD_.O = nh_->param<double>("C_D_O", 0.03);
    CD_.alpha = nh_->param<double>("C_D_alpha", 0.30);
    CD_.beta = nh_->param<double>("C_D_beta", 0.0);
    CD_.p = nh_->param<double>("C_D_p", 0.0437);
    CD_.q = nh_->param<double>("C_D_q", 0.0);
    CD_.r = nh_->param<double>("C_D_r", 0.0);
    CD_.delta_a = nh_->param<double>("C_D_delta_a", 0.0);
    CD_.delta_e = nh_->param<double>("C_D_delta_e", 0.0);
    CD_.delta_r = nh_->param<double>("C_D_delta_r", 0.0);

    // ell Params (x axis moment)
    Cell_.O = nh_->param<double>("C_ell_O", 0.0);
    Cell_.alpha = nh_->param<double>("C_ell_alpha", 0.00);
    Cell_.beta = nh_->param<double>("C_ell_beta", -0.12);
    Cell_.p = nh_->param<double>("C_ell_p", -0.26);
    Cell_.q = nh_->param<double>("C_ell_q", 0.0);
    Cell_.r = nh_->param<double>("C_ell_r", 0.14);
    Cell_.delta_a = nh_->param<double>("C_ell_delta_a", 0.08);
    Cell_.delta_e = nh_->param<double>("C_ell_delta_e", 0.0);
    Cell_.delta_r = nh_->param<double>("C_ell_delta_r", 0.105);

    // m Params (y axis moment)
    Cm_.O = nh_->param<double>("C_m_O", -0.02338);
    Cm_.alpha = nh_->param<double>("C_m_alpha", -0.38);
    Cm_.beta = nh_->param<double>("C_m_beta", 0.0);
    Cm_.p = nh_->param<double>("C_m_p", 0.0);
    Cm_.q = nh_->param<double>("C_m_q", -3.6);
    Cm_.r = nh_->param<double>("C_m_r", 0.0);
    Cm_.delta_a = nh_->param<double>("C_m_delta_a", 0.0);
    Cm_.delta_e = nh_->param<double>("C_m_delta_e", -0.5);
    Cm_.delta_r = nh_->param<double>("C_m_delta_r", 0.0);

    // n Params (z axis moment)
    Cn_.O = nh_->param<double>("C_n_O", 0.0);
    Cn_.alpha = nh_->param<double>("C_n_alpha", 0.0);
    Cn_.beta = nh_->param<double>("C_n_beta", 0.25);
    Cn_.p = nh_->param<double>("C_n_p", 0.022);
    Cn_.q = nh_->param<double>("C_n_q", 0.0);
    Cn_.r = nh_->param<double>("C_n_r", -0.35);
    Cn_.delta_a = nh_->param<double>("C_n_delta_a", 0.06);
    Cn_.delta_e = nh_->param<double>("C_n_delta_e", 0.0);
    Cn_.delta_r = nh_->param<double>("C_n_delta_r", -0.032);

    // Y Params (Sideslip Forces)
    CY_.O = nh_->param<double>("C_Y_O", 0.0);
    CY_.alpha = nh_->param<double>("C_Y_alpha", 0.00);
    CY_.beta = nh_->param<double>("C_Y_beta", -0.98);
    CY_.p = nh_->param<double>("C_Y_p", 0.0);
    CY_.q = nh_->param<double>("C_Y_q", 0.0);
    CY_.r = nh_->param<double>("C_Y_r", 0.0);
    CY_.delta_a = nh_->param<double>("C_Y_delta_a", 0.0);
    CY_.delta_e = nh_->param<double>("C_Y_delta_e", 0.0);
    CY_.delta_r = nh_->param<double>("C_Y_delta_r", -0.017);
}

Fixedwing::ForcesAndTorques Fixedwing::updateFrocesAndTorques(Pose pos, Velocities vel, int act_cmd[], double sample_time)
{
    delta_.a = (act_cmd[0] - 1500.0)/500;
    delta_.e = (act_cmd[1] - 1500.0)/500;
    delta_.t = (act_cmd[2] - 1000.0)/1000;
    delta_.r = (act_cmd[3] - 1500.0)/500;

    double u = vel.u;
    double v = vel.v;
    double w = vel.w;

    double p = vel.p;
    double q = vel.q;
    double r = vel.r;

    // wind info is available in the wind_ struct
    /// TODO: This is wrong. Wind is being applied in the body frame, not inertial frame
    double ur = u - wind_.N;
    double vr = v - wind_.E;
    double wr = w - wind_.D;

    double Va = sqrt(pow(ur,2.0) + pow(vr,2.0) + pow(wr,2.0));

    Fixedwing::ForcesAndTorques forces;

    // Don't divide by zero, and don't let NaN's get through (sometimes GetRelativeLinearVel returns NaNs)
    if(Va > 0.000001 && std::isfinite(Va))
    {
        /*
       * The following math follows the method described in chapter 4 of
       * Small Unmanned Aircraft: Theory and Practice
       * By Randy Beard and Tim McLain.
       * Look there for a detailed explanation of each line in the rest of this function
       */
        double alpha = atan2(wr , ur);
        double beta = asin(vr/Va);

        double sign = (alpha >= 0? 1: -1);//Sigmoid function
        double sigma_a = (1 + exp(-(wing_.M*(alpha - wing_.alpha0))) + exp((wing_.M*(alpha + wing_.alpha0))))/((1 + exp(-(wing_.M*(alpha - wing_.alpha0))))*(1 + exp((wing_.M*(alpha + wing_.alpha0)))));
        double CL_a = (1 - sigma_a)*(CL_.O + CL_.alpha*alpha) + sigma_a*(2*sign*pow(sin(alpha),2.0)*cos(alpha));
        double AR = (pow(wing_.b, 2.0))/wing_.S;
        double CD_a = CD_.p + ((pow((CL_.O + CL_.alpha*(alpha)),2.0))/(3.14159*0.9*AR));//the const 0.9 in this equation replaces the e (Oswald Factor) variable and may be inaccurate

        double CX_a = -CD_a*cos(alpha) + CL_a*sin(alpha);
        double CX_q_a = -CD_.q*cos(alpha) + CL_.q*sin(alpha);
        double CX_deltaE_a = -CD_.delta_e*cos(alpha) + CL_.delta_e*sin(alpha);

        double CZ_a = -CD_a*sin(alpha) - CL_a*cos(alpha);
        double CZ_q_a = -CD_.q*sin(alpha) - CL_.q*cos(alpha);
        double CZ_deltaE_a = -CD_.delta_e*sin(alpha) - CL_.delta_e*cos(alpha);

        forces.Fx = 0.5*(rho_)*pow(Va,2.0)*wing_.S*(CX_a + (CX_q_a*wing_.c*q)/(2.0*Va) + CX_deltaE_a * delta_.e) + 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t),2.0) - pow(Va,2.0));
        forces.Fy = 0.5*(rho_)*pow(Va,2.0)*wing_.S*(CY_.O + CY_.beta*beta + ((CY_.p*wing_.b*p)/(2.0*Va)) + ((CY_.r*wing_.b*r)/(2.0*Va)) + CY_.delta_a*delta_.a + CY_.delta_r*delta_.r);
        forces.Fz = 0.5*(rho_)*pow(Va,2.0)*wing_.S*(CZ_a + (CZ_q_a*wing_.c*q)/(2.0*Va) + CZ_deltaE_a * delta_.e);

        forces.l = 0.5*(rho_)*pow(Va,2.0)*wing_.S*wing_.b*(Cell_.O + Cell_.beta*beta + (Cell_.p*wing_.b*p)/(2.0*Va) + (Cell_.r*wing_.b*r)/(2.0*Va) + Cell_.delta_a*delta_.a + Cell_.delta_r*delta_.r) - prop_.k_T_P*pow((prop_.k_Omega*delta_.t),2.0);
        forces.m = 0.5*(rho_)*pow(Va,2.0)*wing_.S*wing_.c*(Cm_.O + Cm_.alpha*alpha + (Cm_.q*wing_.c*q)/(2.0*Va) + Cm_.delta_e*delta_.e);
        forces.n = 0.5*(rho_)*pow(Va,2.0)*wing_.S*wing_.b*(Cn_.O + Cn_.beta*beta + (Cn_.p*wing_.b*p)/(2.0*Va) + (Cn_.r*wing_.b*r)/(2.0*Va) + Cn_.delta_a*delta_.a + Cn_.delta_r*delta_.r);
    }
    else
    {
//        if(!std::isfinite(Va))
//        {
//            gzerr << "u = " << u << "\n";
//            gzerr << "v = " << v << "\n";
//            gzerr << "w = " << w << "\n";
//            gzerr << "p = " << p << "\n";
//            gzerr << "q = " << q << "\n";
//            gzerr << "r = " << r << "\n";
//            gzerr << "ur = " << ur << "\n";
//            gzerr << "vr = " << vr << "\n";
//            gzerr << "wr = " << wr << "\n";
//            gzthrow("we have a NaN or an infinity:\n");
//        }
//        else
//        {
            forces.Fx = 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t),2.0));
            forces.Fy = 0.0;
            forces.Fz = 0.0;
            forces.l = 0.0;
            forces.m = 0.0;
            forces.n = 0.0;
//        }
    }

    return forces;
}
