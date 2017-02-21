/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <time.h>
#include <nlopt.hpp>
#include  <omp.h>
#include "solver.h"
#include "svm_grad.h"
#include "sg_filter.h"

using namespace Eigen;
using namespace std;



IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

struct Jacobian_S
{
MatrixXd 	Jacobian[7]; 	// The size of jacobian is based on the need!
MatrixXd 	Jacobian_7[7];	// The size of jacobian is 7\times 3
Vector3d 	Link_pos[7];
//MatrixXd 	Jacobian_Full;
};

struct S_Robot_IK {
	bool jacobian_is_set;
	bool desired_position_is_set;
	bool state_is_set;
	int index;
	int N_links;
	int N_constraints;
	MatrixXd W;
	MatrixXd Jacobian;
	VectorXd U_q;
	VectorXd L_q;
	VectorXd U_Dq;
	VectorXd L_Dq;
	VectorXd U_DDq;
	VectorXd L_DDq;
	VectorXd Desired_end;
	VectorXd q;
	VectorXd Dq;
	Jacobian_S Jacobian_R;
};

typedef struct {
    MatrixXd J;
    VectorXd X;
    VectorXd desired;
} my_object_data;



Vars vars;
Params params;
Workspace work;
Settings settings;


enum Solver_level{Velocity_level=0,Acceleration_level};
enum Solver_type{Numerical=0,Dynamical};
enum Solver_Numerical{Nlopt,CVXgen1,CVXgen2};

int Max_N_robot=3;

bool save_the_performace=true;



class qp_ik_solver
{
public:

	void Initialize(int N_robots, double dt,Solver_type type,Solver_level level,bool Super_constraint);
    void Initialize(int N_robots, double dt,Solver_type type,Solver_level level,bool Super_constraint, string svm_filename);
	void Initialize_robot(int index,int N_links,int N_constraints,MatrixXd W,VectorXd U_q, VectorXd L_q,	VectorXd U_Dq,VectorXd L_Dq,VectorXd U_DDq,VectorXd L_DDq);
	void Initialize_robot(int index,int N_links,int N_constraints,MatrixXd W,VectorXd U_q, VectorXd L_q,	VectorXd U_Dq,VectorXd L_Dq);
	void Finalize_Initialization();
	void set_jacobian(int index,MatrixXd Jacobian);
	void set_jacobian_links(int index,Jacobian_S Jacobian);
	void set_desired(int index,VectorXd Desired_end);
	void set_state(int index,VectorXd q,VectorXd Dq);
	void get_state(int index, VectorXd &Dq);
    void get_gamma(double &gamma);
	void Solve();

private:
	void 		ERROR();
	void 		Print_Robot(S_Robot_IK Robot);
	VectorXd	Omega_projector(VectorXd Input,VectorXd Upper,VectorXd Lower);

	inline	void 		Construct_vel();
	inline	void		Construct_boundaries_vel();

	inline	void		Construct_collision_boundaries();



	inline	void		restart_the_robots();

	inline 	void		Check_feasibility(VectorXd U);

	inline 	void		load_default_data();


	double 		dt_;
	Solver_type Solver_type_;
	Solver_level Solver_level_;
	Solver_Numerical 	Solver_N_Type;
	S_Robot_IK 	*Robots_;
	int 		N_robots_;
	int			Dimension_constraint_;
	int			Dimension_q_;


	MatrixXd	W_;
	MatrixXd	M_;
	MatrixXd	J_;
	VectorXd	b_;
	VectorXd	P_Omega_;

	VectorXd	U_plus_;
	VectorXd	U_minus_;
	VectorXd	Theta_plus_;
	VectorXd	Theta_minus_;

	MatrixXd	I_;
	MatrixXd	Handle_M_I_;
	VectorXd	Handle_projection;


	VectorXd	U_;
	VectorXd	DU_;


	VectorXd	Const_Upper_;
	VectorXd	Const_Lower_;

	double 		mu_p_;
	double 		eta_p_;

    SVMGrad     svmBoundary_;
	double		Gamma_;
	VectorXd	DGamma_;
    bool        considerCollision;
    double      lambda;

	bool 		Super_constraint_;


	double t1;
	clock_t duration;

	MatrixXd 	G_QP_;
	VectorXd 	g0_QP_;
	MatrixXd 	CE_QP_;
	VectorXd 	ce0_QP_;
/*	MatrixXd 	CI_QP_;
	VectorXd 	ci0_QP_;*/
	VectorXd 	X_QP_;

	my_object_data data_object;
	nlopt::opt *opt;
	std::vector<double> lb_;
	std::vector<double> ub_;


    ofstream 	        myfile;

    SGF::SavitzkyGolayFilter *filter;
    SGF::Vec inp;
    SGF::Vec outp;
    int ret_code;

};

