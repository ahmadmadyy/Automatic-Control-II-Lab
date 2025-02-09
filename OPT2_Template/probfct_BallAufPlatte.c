/* This file is part of GRAMPC - (https://sourceforge.net/projects/grampc/)
*
* GRAMPC -- A software framework for embedded nonlinear model predictive
* control using a gradient-based augmented Lagrangian approach
*
* Copyright (C) 2014-2018 by Tobias Englert, Knut Graichen, Felix Mesmer,
* Soenke Rhein, Andreas Voelz, Bartosz Kaepernick (<v2.0), Tilman Utz (<v2.0).
* Developed at the Institute of Measurement, Control, and Microtechnology,
* Ulm University. All rights reserved.
*
* GRAMPC is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of
* the License, or (at your option) any later version.
*
* GRAMPC is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with GRAMPC. If not, see <http://www.gnu.org/licenses/>
*
*
*
*
*
*
*
* This probfct file provides an interface to GRAMPC. The most general
* formulation of the optimal control problem (OCP), that can be sovled
* by GRAMPC, has the following structure
*                                            _T
*                                          /
*      min    J(u,p,T;x0) = V(T,x(T),p) + / l(t,x(t),u(t),p) dt
*   u(.),p,T                            _/
*                                      0
*             .
*      s.t.   x(t) = f(t0+t,x(t),u(t),p), x(0) = x0
*             g(t,x(t),u(t),p)  = 0,   gT(T,x(T),p)  = 0
*             h(t,x(t),u(t),p) <= 0,   hT(T,x(t),p) <= 0
*             u_min <= u(t) <= u_max
*             p_min <= p <= p_max
*             T_min <= T <= T_max
*
*
*/

#include "probfct.h"

#define N_PCOST 9
#define N_PTRAJ 4

#define g 9.81
#define B 0.714285714285714

/** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
    inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    //TODO
    *Nx = 8;
	*Nu = 2;
	*Np = 0;
	*Nh = 8;
	*Ng = 0;
	*NgT = 0;
	*NhT = 0;
}


/** System function f(t,x,u,p,userparam)
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    ctypeRNum* controlType = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ;
	
     //TODO
	out[0] = x[1];
	out[1] = B * (x[0] * x[3] * x[3] - g * sin(x[2]));
    out[2] = x[3];
    out[3] = u[0];
	out[4] = x[5];
    out[5] = B * (x[4] * x[7] * x[7] - g * sin(x[6]));
    out[6] = x[7];
	out[7] = u[1];
}


/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    ctypeRNum* controlType = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ;
	
	out[0] = B * x[3] * x[3] * vec[1];
	out[1] = vec[0];
    out[2] = (-1 * B * g * cos(x[2])) * vec[1]; //
    out[3] = (2 * B * x[0] * x[3]) * vec[1] + vec[2]; //
	out[4] = B * x[7] * x[7] * vec[5];
    out[5] = vec[4];
    out[6] = (-1 * B * g * cos(x[6])) * vec[5]; //;
	out[7] = (2 * B * x[4] * x[7]) * vec[5] + vec[6]; //;
}


/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
	//TODO
	out[0] = vec[3];
	out[1] = vec[7];
}


/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}


/** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam) 
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    ctypeRNum *pCost = (ctypeRNum*)userparam;
    ctypeRNum *pTraj = (ctypeRNum*)userparam + N_PCOST;
    ctypeRNum *controlType = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ;
    ctypeRNum *t_ext = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ+1;

	typeRNum* Q = ((typeRNum*)userparam);
    typeRNum* R = Q + 7;
    typeRNum temp = 0;
    typeRNum tempx = 0;
    typeRNum tempy = 0;

    if (controlType[0]==0)
    {
        out[0] += 0.5 * (x[0] - xdes[0]) * Q[0] * (x[0] - xdes[0]);
        out[0] += 0.5 * (x[1] - xdes[1]) * Q[2] * (x[1] - xdes[1]);
        out[0] += 0.5 * (x[2] - xdes[2]) * Q[3] * (x[2] - xdes[2]);
        out[0] += 0.5 * (x[3] - xdes[3]) * Q[4] * (x[3] - xdes[3]);
        out[0] += 0.5 * (x[4] - xdes[4]) * Q[1] * (x[4] - xdes[4]);
        out[0] += 0.5 * (x[5] - xdes[5]) * Q[2] * (x[5] - xdes[5]);
        out[0] += 0.5 * (x[6] - xdes[6]) * Q[5] * (x[6] - xdes[6]);
        out[0] += 0.5 * (x[7] - xdes[7]) * Q[6] * (x[7] - xdes[7]);
        
        for(typeInt i = 0; i < 2; ++i)
        {
            out[0] += 0.5 * (u[i] - udes[i]) * R[i] * (u[i] - udes[i]);
        }
    
    }
    else if (controlType[0]==1)
    {
        tempx = pTraj[0] * sin(pTraj[2] * pTraj[3] * (t_ext[0]+t));
        tempy = pTraj[1] * cos(pTraj[3] * (t_ext[0]+t));
        out[0] += 0.5 * (x[0] - tempx) * Q[0] * (x[0] - tempx);
        out[0] += 0.5 * (x[1] - xdes[1]) * Q[2] * (x[1] - xdes[1]);
        out[0] += 0.5 * (x[2] - xdes[2]) * Q[3] * (x[2] - xdes[2]);
        out[0] += 0.5 * (x[3] - xdes[3]) * Q[4] * (x[3] - xdes[3]);
        out[0] += 0.5 * (x[4] - tempy) * Q[1] * (x[4] - tempy);
        out[0] += 0.5 * (x[5] - xdes[5]) * Q[2] * (x[5] - xdes[5]);
        out[0] += 0.5 * (x[6] - xdes[6]) * Q[5] * (x[6] - xdes[6]);
        out[0] += 0.5 * (x[7] - xdes[7]) * Q[6] * (x[7] - xdes[7]);

        for(typeInt i = 0; i < 2; ++i)
        {
            out[0] += 0.5 * (u[i] - udes[i]) * R[i] * (u[i] - udes[i]);
        }
    } 
}


/** Gradient dl/dx **/ 
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    ctypeRNum *pCost = (ctypeRNum*)userparam;
    ctypeRNum *pTraj = (ctypeRNum*)userparam + N_PCOST;
    ctypeRNum *controlType = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ;
    ctypeRNum *t_ext = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ + 1;

    typeRNum* Q = ((typeRNum*)userparam); 
    typeRNum temp = 0;
    typeRNum tempx = 0;
    typeRNum tempy = 0;

    if (controlType[0] == 0)
    {
        out[0] = Q[0] * (x[0] - xdes[0]);
        out[1] = Q[2] * (x[1] - xdes[1]);
        out[2] = Q[3] * (x[2] - xdes[2]);
        out[3] = Q[4] * (x[3] - xdes[3]);
        out[4] = Q[1] * (x[4] - xdes[4]);
        out[5] = Q[2] * (x[5] - xdes[5]);
        out[6] = Q[5] * (x[6] - xdes[6]);
        out[7] = Q[6] * (x[7] - xdes[7]);
    }
    else if (controlType[0] == 1)
    {
        tempx = pTraj[0] * sin(pTraj[2] * pTraj[3] * (t_ext[0]+t));
        tempy = pTraj[1] * cos(pTraj[3] * (t_ext[0]+t));
        out[0] = Q[0] * (x[0] - tempx);
        out[1] = Q[2] * (x[1] - xdes[1]);
        out[2] = Q[3] * (x[2] - xdes[2]);
        out[3] = Q[4] * (x[3] - xdes[3]);
        out[4] = Q[1] * (x[4] - tempy);
        out[5] = Q[2] * (x[5] - xdes[5]);
        out[6] = Q[5] * (x[6] - xdes[6]);
        out[7] = Q[6] * (x[7] - xdes[7]);
    }
}




/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    ctypeRNum *pCost = (ctypeRNum*)userparam;
    ctypeRNum *pTraj = (ctypeRNum*)userparam + N_PCOST;
    ctypeRNum *controlType = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ;
    ctypeRNum *t_ext = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ+1;

    typeRNum* Q = ((typeRNum*)userparam);
    typeRNum* R = Q + 7;
    
  for(typeInt i = 0; i < 2; ++i)
  {
      out[i] = R[i] * (u[i] - udes[i]); 
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
    ctypeRNum *pCost = (ctypeRNum*)userparam;
    ctypeRNum *pTraj = (ctypeRNum*)userparam + N_PCOST;
    ctypeRNum *controlType = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ;
    ctypeRNum *t_ext = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ+1;

    typeRNum* Q = ((typeRNum*)userparam);
    typeRNum* S = Q + 11;
    typeRNum temp = 0;
    typeRNum tempx = 0;
    typeRNum tempy = 0;
    

   //TODO
    
    if (controlType[0]==0)
    {
        out[0] += 0.5 * (x[0] - xdes[0]) * S[0] * (x[0] - xdes[0]);
        out[0] += 0.5 * (x[1] - xdes[1]) * S[2] * (x[1] - xdes[1]);
        out[0] += 0.5 * (x[2] - xdes[2]) * S[3] * (x[2] - xdes[2]);
        out[0] += 0.5 * (x[3] - xdes[3]) * S[4] * (x[3] - xdes[3]);
        out[0] += 0.5 * (x[4] - xdes[4]) * S[1] * (x[4] - xdes[4]);
        out[0] += 0.5 * (x[5] - xdes[5]) * S[2] * (x[5] - xdes[5]);
        out[0] += 0.5 * (x[6] - xdes[6]) * S[5] * (x[6] - xdes[6]);
        out[0] += 0.5 * (x[7] - xdes[7]) * S[6] * (x[7] - xdes[7]);
    }
    else if (controlType[0]==1)
    {
        tempx = pTraj[0] * sin(pTraj[2] * pTraj[3] * (t_ext[0]+T));
        tempy = pTraj[1] * cos(pTraj[3] * (t_ext[0]+T));
        out[0] += 0.5 * (x[0] - tempx) * S[0] * (x[0] - tempx);
        out[0] += 0.5 * (x[1] - xdes[1]) * S[2] * (x[1] - xdes[1]);
        out[0] += 0.5 * (x[2] - xdes[2]) * S[3] * (x[2] - xdes[2]);
        out[0] += 0.5 * (x[3] - xdes[3]) * S[4] * (x[3] - xdes[3]);
        out[0] += 0.5 * (x[4] - tempy) * S[1] * (x[4] - tempy);
        out[0] += 0.5 * (x[5] - xdes[5]) * S[2] * (x[5] - xdes[5]);
        out[0] += 0.5 * (x[6] - xdes[6]) * S[5] * (x[6] - xdes[6]);
        out[0] += 0.5 * (x[7] - xdes[7]) * S[6] * (x[7] - xdes[7]);
     }

}


/** Gradient dV/dx **/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
    ctypeRNum *pCost = (ctypeRNum*)userparam;
    ctypeRNum *pTraj = (ctypeRNum*)userparam + N_PCOST;
    ctypeRNum *controlType = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ;
    ctypeRNum *t_ext = (ctypeRNum*)userparam + N_PCOST + N_PTRAJ + 1;

    typeRNum* S = (typeRNum*)userparam + 11;
    typeRNum temp = 0;
    typeRNum tempx = 0;
    typeRNum tempy = 0;

    if (controlType[0] == 0)
    {
        out[0] = S[0] * (x[0] - xdes[0]);
        out[1] = S[2] * (x[1] - xdes[1]);
        out[2] = S[3] * (x[2] - xdes[2]);
        out[3] = S[4] * (x[3] - xdes[3]);
        out[4] = S[1] * (x[4] - xdes[4]);
        out[5] = S[2] * (x[5] - xdes[5]);
        out[6] = S[5] * (x[6] - xdes[6]);
        out[7] = S[6] * (x[7] - xdes[7]);
    }
    else if (controlType[0] == 1)
    {
        tempx = pTraj[0] * sin(pTraj[2] * pTraj[3] * (t_ext[0]+T));
        tempy = pTraj[1] * cos(pTraj[3] * (t_ext[0]+T));
        out[0] = S[0] * (x[0] - tempx);
        out[1] = S[2] * (x[1] - xdes[1]);
        out[2] = S[3] * (x[2] - xdes[2]);
        out[3] = S[4] * (x[3] - xdes[3]);
        out[4] = S[1] * (x[4] - tempy);
        out[5] = S[2] * (x[5] - xdes[5]);
        out[6] = S[5] * (x[6] - xdes[6]);
        out[7] = S[6] * (x[7] - xdes[7]);
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


/** Inequality constraints h(t,x(t),u(t),p,uperparam) <= 0 
    ------------------------------------------------------ **/
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    out[0] = x[0] - 0.1775;      
    out[1] = -x[0] - 0.1775;     
    out[2] = x[4] - 0.2365;      
    out[3] = -x[4] - 0.2365;   
    out[4] = x[2] - 13;
    out[5] = -x[2] - 13;
    out[6] = x[6] - 14;
    out[7] = -x[6] - 14;
}


/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    //TODO
	out[0] += vec[0];
    out[0] += - vec[1];
    out[4] += vec[2];
    out[4] += - vec[3];
    out[2] += vec[4];
    out[2] += - vec[5];
    out[6] += vec[6];
    out[6] += - vec[7];
   
}


/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    //TODO
	for (int i = 0; i < 2; i++) {
        out[i] = 0;
    }
;
    //...
}


/** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
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


/** Terminal equality constraints gT(T,x(T),p,uperparam) = 0 
    ----------------------------------------------------------- **/
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

