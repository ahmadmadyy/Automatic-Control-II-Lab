/* This file is part of GRAMPC - (https://sourceforge.net/projects/grampc/)
 *
 * GRAMPC -- A software framework for embedded nonlinear model predictive
 * control using a gradient-based augmented Lagrangian approach
 *
 * Copyright 2014-2019 by Tobias Englert, Knut Graichen, Felix Mesmer,
 * Soenke Rhein, Andreas Voelz, Bartosz Kaepernick (<v2.0), Tilman Utz (<v2.0).
 * All rights reserved.
 *
 * GRAMPC is distributed under the BSD-3-Clause license, see LICENSE.txt
 *
 *
 *
 *
 *
 *
 *
 *
 * This probfct-file describes the crane 2D problem from
 * Kapernick, B., Graichen, K.: Model predictive control of an overhead
 * crane using constraint substitution. In: Proc. American Control 
 * Conference (ACC), pp. 3973-3978, 2013.
 *
 *                                           _T
 *                                          /
 *      min    J(u,p,T;x0) = V(T,x(T),p) + / l(t,x(t),u(t),p) dt
 *   u(.),p,T                            _/
 *                                      0
 *             .
 *      s.t.   x(t) = f(t0+t,x(t),u(t),p), x(0) = x0
 *             h(x)  <= 0
 *             u_min <= u(t) <= u_max
 *
 */

#include "probfct.h"

 /* Data type-dependent functions */
#if USE_typeRNum == USE_FLOAT
#define SIN(a)		sinf(a)
#define COS(a)		cosf(a)
#else
#define SIN(a)		sin(a)
#define COS(a)		cos(a)
#endif 

 /* square macro */
#define POW2(a) ((a)*(a))

 /* gravitation constant */
#define GRAV ((typeRNum) 9.81)

#define N_PCOST 10

/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng), 
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    //TODO
    *Nx = 6;
	*Nu = 2;
	*Np = 0;
	*Nh = 4;
	*Ng = 0;
	*NgT = 0;
	*NhT = 0;
}


/** System function f(t,x,u,p,userparam) 
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    //TODO
	out[0] = x[1];
	out[1] = u[0];
    out[2] = x[3];
    out[3] = u[1];
    out[4] = x[5];
    out[5] = ((-GRAV * sin(x[4])) - (u[0] * cos(x[4])) - (2 * x[3] * x[5])) / x[2] ;
}
/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
	//TODO
    out[0] = 0;
	out[1] = vec[0];
    out[2] = -(vec[5]/(x[2]*x[2])) * ((-GRAV * sin(x[4])) - (u[0] * cos(x[4])) - (2 * x[3] * x[5]));
    out[3] = vec[2] - (2 * vec[5] * x[5]) / x[2];
    out[4] = (-1 / x[2]) * (vec[5] * ((GRAV * cos(x[4])) - (u[0] * sin(x[4]))));
    out[5] = vec[4] - (2 * vec[5] * x[3]) / x[2];
}
/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
	//TODO
	out[0] = vec[1] - ((cos(x[4]))*vec[5]) / x[2];
	out[1] = vec[3];
}
/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}


/** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam) 
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
	typeRNum* Q = ((typeRNum*)userparam);
    typeRNum* R = Q + 6;
    
    out[0] = 0;
    typeRNum temp = x[0];
    for(typeInt i = 0; i < 6; ++i)
    {
        if(i==0)
        {
            temp = x[0] + x[2] * sin(x[4]);
            out[0] += 0.5 * (temp - xdes[i]) * Q[i] * (temp - xdes[i]); 
        }
        else if(i==2) 
        {
            temp = 0.72 - x[2] * cos(x[4]);
            out[0] += 0.5 * (temp - xdes[i]) * Q[i] * (temp - xdes[i]);
        }
        else 
        {
            out[0] += 0.5 * (x[i] - xdes[i]) * Q[i] * (x[i] - xdes[i]); 
        }
    }
    for(typeInt i = 0; i < 2; ++i)
    {
        out[0] += 0.5 * (u[i] - udes[i]) * R[i] * (u[i] - udes[i]);
    }
}
/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    typeRNum* Q = ((typeRNum*)userparam); // State weighting matrix
    typeRNum temp = 0;

    // Loop through all state variables
    for (typeInt i = 0; i < 6; ++i)
    {
        if (i == 0)
        {
            // For x_C = x[0], transformed as x_C + l * sin(theta)
            temp = x[0] + x[2] * sin(x[4]);
            out[i] = Q[0] * (temp - xdes[0]); // Derivative of cost w.r.t. x[0]
        }
        else if (i == 2)
        {
            // For l = x[2], transformed as x_C + l * sin(theta) and 0.72 - l * cos(theta)
            typeRNum dQ0_term = sin(x[4]); // Derivative of the Q[0]-term contribution
            typeRNum dQ2_term = -cos(x[4]); // Derivative of the Q[2]-term contribution
            temp = 0.72 - x[2] * cos(x[4]);

            // Combine both contributions
            out[i] = Q[0] * (x[0] + x[2] * sin(x[4]) - xdes[0]) * dQ0_term +
                     Q[2] * (temp - xdes[2]) * dQ2_term;
        }
        else if (i == 4)
        {
            // For theta = x[4], contributions from Q[0] (x_C + l * sin(theta)) and Q[2] (0.72 - l * cos(theta))
            typeRNum dQ0_term_theta = x[2] * cos(x[4]); // Derivative of Q[0]-term w.r.t theta
            typeRNum dQ2_term_theta = x[2] * sin(x[4]); // Derivative of Q[2]-term w.r.t theta

            out[i] = Q[0] * (x[0] + x[2] * sin(x[4]) - xdes[0]) * dQ0_term_theta +
                     Q[2] * (0.72 - x[2] * cos(x[4]) - xdes[2]) * dQ2_term_theta +
                     Q[4] * (x[4] - xdes[4]); // Add the direct derivative of Q[4]-term
        }
        else
        {
            // For all other state variables, standard quadratic term
            out[i] = Q[i] * (x[i] - xdes[i]);
        }
    }
}

/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    typeRNum* Q = ((typeRNum*)userparam);
    typeRNum* R = Q + 6;
    
    for(typeInt i = 0; i < 2; ++i)
    {
        out[i] = R[i] * (u[i] - udes[i]);
        //out[i] = 0;  
    }
}
/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
}


/** Terminal cost V(T,x(T),p,xdes,userparam) 
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    typeRNum* S = (typeRNum*)userparam + 10;
    
    out[0] = 0;
    typeRNum temp = x[0];
    for(typeInt i = 0; i < 6; ++i)
    {
        //out[0] += 0.5 * (x[i] - xdes[i]) * S[i] * (x[i] - xdes[i]);
        if(i==0)
        {
            temp = x[0] + x[2] * sin(x[4]);
            out[0] += 0.5 * (temp - xdes[i]) * S[i] * (temp - xdes[i]); 
        }
        else if(i==2) 
        {
            temp = 0.72 - x[2] * cos(x[4]);
            out[0] += 0.5 * (temp - xdes[i]) * S[i] * (temp - xdes[i]);
        }
        else 
        {
            out[0] += 0.5 * (x[i] - xdes[i]) * S[i] * (x[i] - xdes[i]);
        }
    }
}
/** Gradient dV/dx **/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    typeRNum* S = (typeRNum*)userparam + 10; // Terminal weighting matrix
    typeRNum temp = 0;

    // Loop through all state variables
    for (typeInt i = 0; i < 6; ++i)
    {
        if (i == 0)
        {
            // For x_C = x[0], transformed as x_C + l * sin(theta)
            temp = x[0] + x[2] * sin(x[4]);
            out[i] = S[0] * (temp - xdes[0]); // Derivative of cost w.r.t. x[0]
        }
        else if (i == 2)
        {
            // For l = x[2], transformed as x_C + l * sin(theta) and 0.72 - l * cos(theta)
            typeRNum dS0_term = sin(x[4]); // Derivative of S[0]-term contribution
            typeRNum dS2_term = -cos(x[4]); // Derivative of S[2]-term contribution
            temp = 0.72 - x[2] * cos(x[4]);

            // Combine both contributions
            out[i] = S[0] * (x[0] + x[2] * sin(x[4]) - xdes[0]) * dS0_term +
                     S[2] * (temp - xdes[2]) * dS2_term;
        }
        else if (i == 4)
        {
            // For theta = x[4], contributions from S[0] (x_C + l * sin(theta)) and S[2] (0.72 - l * cos(theta))
            typeRNum dS0_term_theta = x[2] * cos(x[4]); // Derivative of S[0]-term w.r.t theta
            typeRNum dS2_term_theta = x[2] * sin(x[4]); // Derivative of S[2]-term w.r.t theta

            out[i] = S[0] * (x[0] + x[2] * sin(x[4]) - xdes[0]) * dS0_term_theta +
                     S[2] * (0.72 - x[2] * cos(x[4]) - xdes[2]) * dS2_term_theta +
                     S[4] * (x[4] - xdes[4]); // Add the direct derivative of S[4]-term
        }
        else
        {
            // For all other state variables, standard quadratic term
            out[i] = S[i] * (x[i] - xdes[i]);
        }
    }
}


/** Gradient dV/dp **/
void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}
/** Gradient dV/dT **/
void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}


/** Equality constraints g(t,x(t),u(t),p,uperparam) = 0 
    --------------------------------------------------- **/
void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Inequality constraints h(t,x(t),u(t),p,uperparam) <= 0 
    ------------------------------------------------------ **/
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    // State Constraints
    out[0] = x[0] - 0.6;       // h0 = x - 0.6
    out[1] = -x[0] - 0.6;      // h1 = -x - 0.6
    out[2] = x[2] - 0.8;       // h2 = l - 0.8
    out[3] = -x[2] + 0.25;     // h3 = -l + 0.25
}

/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    // Initialize output to zero
    for (int i = 0; i < 6; i++) {
        out[i] = 0;
    }

    // Contributions from each constraint
    out[0] += vec[0] * 1;       // h0 w.r.t x[0]
    out[0] += vec[1] * -1;      // h1 w.r.t x[0]
    out[2] += vec[2] * 1;       // h2 w.r.t x[2]
    out[2] += vec[3] * -1;      // h3 w.r.t x[2]
}

/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    // Initialize output to zero (no contributions from inputs)
    for (int i = 0; i < 2; i++) {
        out[i] = 0;
    }
}

/** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Terminal equality constraints gT(T,x(T),p,uperparam) = 0 
    -------------------------------------------------------- **/
void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Terminal inequality constraints hT(T,x(T),p,uperparam) <= 0 
    ----------------------------------------------------------- **/
void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dx multiplied by vector vec, i.e. (dhT/dx)^T*vec or vec^T*(dhT/dx) **/
void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dp multiplied by vector vec, i.e. (dhT/dp)^T*vec or vec^T*(dhT/dp) **/
void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dT multiplied by vector vec, i.e. (dhT/dT)^T*vec or vec^T*(dhT/dT) **/
void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Additional functions required for semi-implicit systems 
    M*dx/dt(t) = f(t0+t,x(t),u(t),p) using the solver RODAS 
    ------------------------------------------------------- **/
/** Jacobian df/dx in vector form (column-wise) **/
void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dx in vector form (column-wise) **/
void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dt **/
void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian d(dH/dx)/dt  **/
void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *vec, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mfct(typeRNum *out, typeUSERPARAM *userparam)
{
}
/** Transposed mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mtrans(typeRNum *out, typeUSERPARAM *userparam)
{
}
