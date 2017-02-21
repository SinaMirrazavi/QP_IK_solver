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


#include "qp_ik_solver.h"


int order = 3;
int winlen = 20;
SGF::real sample_time = 0.002;



double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
	int size=x.size();
	my_object_data *d = reinterpret_cast<my_object_data*>(my_func_data);
	VectorXd X = d->X;
	for (int i=0;i<X.rows();i++){X(i)=x[i];}
	if (!grad.empty()) {
		for (int i=0;i<size;i++)
		{
			grad[i] = 1;
		}
	}
	return ((d->J)*X-d->desired).norm();
}

/*double myvconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
	    my_constraint_data *d = reinterpret_cast<my_constraint_data*>(data);
    my_constraint_data *d = reinterpret_cast<my_constraint_data*>(data);
    MatrixXd J = d->J_row;
    VectorXd X = d->X;
    double desired =d->desired;

    cout<<"J "<<J<<endl;
    cout<<"X "<<X<<endl;
    for (int i=0;i<X.rows();i++){X(i)=x[i];}
	if (!grad.empty()) {
		for (int i=0; i<J.cols();i++)
		{
			grad[i]=J(0,i);
		}
	}
	MatrixXd result=J*X;
	return (result(0,0)-desired);
}*/

void qp_ik_solver::Initialize(int N_robots,double dt,Solver_type type,Solver_level level,bool Super_constraint)
{
	/* Declare the number of the robots
	 * sample time (dt)
	 * and Type of the solver*/

	N_robots_=N_robots;
	dt_=dt;
	Robots_=new S_Robot_IK[N_robots_];
	Solver_level_=level;
	Solver_type_=type;
	mu_p_=20;
	eta_p_=0.9;
	Super_constraint_=Super_constraint;
	Solver_N_Type=CVXgen1;
	if (save_the_performace){	myfile.open ("IK_solver_performace_Dynamical.txt");}
	considerCollision = false;

}


void qp_ik_solver::Initialize(int N_robots,double dt,Solver_type type,Solver_level level,bool Super_constraint, string svm_filename)
{
	/* Declare the number of the robots
	 * sample time (dt)
	 * Type of the solver
	 * Collision Boundary SVM filename (full path) */

	N_robots_=N_robots;
	dt_=dt;
	Robots_=new S_Robot_IK[N_robots_];
	Solver_level_=level;
	Solver_type_=type;
	mu_p_=20;
	eta_p_=0.9;
	Super_constraint_=Super_constraint;
	Solver_N_Type=CVXgen1;
	if (save_the_performace){	myfile.open ("IK_solver_performace_Dynamical.txt");}

	svmBoundary_.loadModel(svm_filename);
	svmBoundary_.preComputeKernel(true);
	considerCollision = true;
    lambda = 10;

}



void qp_ik_solver::Initialize_robot(int index,int N_links,int N_constraints,MatrixXd W,VectorXd U_q, VectorXd L_q,VectorXd U_Dq,VectorXd L_Dq,VectorXd U_DDq,VectorXd L_DDq)
{
	/*	Declare the parameters of   index th robot.
	 N_links is number of the links.
	 N_constraints is the number of the constraints of the IK solver.
	 U_p and L_p are the upper and the lower bounds of the joints' positions.
 	 U_Dp and L_Dp are the upper and the lower bounds of the joints' velocities.
 	 U_DDp and L_DDp are the upper and the lower bounds of the joints' accelerations.
 	 W is the weight.
	 */


	Initialize_robot(index,N_links,N_constraints,W,U_q, L_q,U_Dq,L_Dq);
	if (Solver_level_==Acceleration_level)
	{
		if ((U_DDq.rows()!=N_links)||(L_DDq.rows()!=N_links))
		{
			cout<<"Initialization of "<<index<<"th robot is wrong."<<endl;
			cout<<"N_links "<<N_links<<endl;
			cout<<"U_DDp: "<<endl;cout<<U_DDq<<endl;
			cout<<"U_DDp: "<<endl;cout<<U_DDq<<endl;
			cout<<"Initialization of "<<index<<"th robot is wrong."<<endl;
			ERROR();
		}
		else
		{
			Robots_[index].U_DDq=U_DDq;
			Robots_[index].L_DDq=L_DDq;
		}
	}
	Print_Robot(Robots_[index]);

}
void qp_ik_solver::Initialize_robot(int index,int N_links,int N_constraints,MatrixXd W,VectorXd U_q, VectorXd L_q,VectorXd U_Dq,VectorXd L_Dq)
{
	/*	Declare the parameters of   index th robot.
	 N_links is number of the links.
	 N_constraints is the number of the constraints of the IK solver.
	 U_p and L_p are the upper and the lower bounds of the joints' positions.
 	 U_Dp and L_Dp are the upper and the lower bounds of the joints' velocities.
 	 U_DDp and L_DDp are the upper and the lower bounds of the joints' accelerations.
 	 W is the weight.
	 */
	if (index>N_robots_-1)
	{
		cout<<"Initialization of "<<index<<"th robot is wrong."<<endl;
		cout<<"index "<<index<<" Max robot Number "<<N_robots_-1<<endl;
		ERROR();
	}

	Robots_[index].index=index;
	Robots_[index].N_links=N_links;

	Robots_[index].N_constraints=N_constraints;

	Robots_[index].W.resize(Robots_[index].N_links,Robots_[index].N_links);
	Robots_[index].Jacobian.resize(Robots_[index].N_constraints,Robots_[index].N_links);
	Robots_[index].Desired_end.resize(Robots_[index].N_constraints);

	Robots_[index].U_q.resize(Robots_[index].N_links);
	Robots_[index].L_q.resize(Robots_[index].N_links);
	Robots_[index].U_Dq.resize(Robots_[index].N_links);
	Robots_[index].L_Dq.resize(Robots_[index].N_links);
	Robots_[index].U_DDq.resize(Robots_[index].N_links);
	Robots_[index].L_DDq.resize(Robots_[index].N_links);
	Robots_[index].q.resize(Robots_[index].N_links);
	Robots_[index].Dq.resize(Robots_[index].N_links);

	for (int i=0;i<7;i++)
	{
		Robots_[index].Jacobian_R.Jacobian[i].resize(3,1+i);Robots_[index].Jacobian_R.Jacobian[i].setZero();
		Robots_[index].Jacobian_R.Jacobian_7[i].resize(3,7);Robots_[index].Jacobian_R.Jacobian_7[i].setZero();
	}

	if ((U_q.rows()!=N_links)||(L_q.rows()!=N_links)||(L_Dq.rows()!=N_links)||(U_Dq.rows()!=N_links)||(W.cols()!=N_links)||(W.rows()!=N_links))
	{
		cout<<"Initialization of "<<index<<"th robot is wrong."<<endl;
		cout<<"N_links "<<N_links<<endl;
		cout<<"U_p: "<<endl;cout<<U_q<<endl;
		cout<<"L_p: "<<endl;cout<<L_q<<endl;
		cout<<"U_Dp:"<<endl;cout<<U_Dq<<endl;
		cout<<"L_Dp:"<<endl;cout<<L_Dq<<endl;
		cout<<"W:  	"<<endl;cout<<W<<endl;
		cout<<"Initialization of "<<index<<"th robot is wrong."<<endl;
		ERROR();
	}
	else
	{
		Robots_[index].W=W;
		Robots_[index].U_q=U_q;
		Robots_[index].L_q=L_q;
		Robots_[index].U_Dq=U_Dq;
		Robots_[index].L_Dq=L_Dq;
		Robots_[index].L_DDq.setZero();
		Robots_[index].U_DDq.setZero();
	}


	//Print_Robot(Robots_[index]);

}
void qp_ik_solver::Finalize_Initialization()
{
	/*	Initializing the variable of the main solver
	 * 	 												*/
	int Dimension_q=0;
	for (int i=0;i<N_robots_;i++)
	{
		Dimension_q=Dimension_q+Robots_[i].N_links;
	}
	int Dimension_constraint=0;
	for (int i=0;i<N_robots_;i++)
	{
		Dimension_constraint=Dimension_constraint+Robots_[i].N_constraints;
	}
	Dimension_constraint_=Dimension_constraint;
	Dimension_q_=Dimension_q;
	W_.resize(Dimension_q_,Dimension_q_);															W_.setZero();
	J_.resize(Dimension_constraint_,Dimension_q_);													J_.setZero();
	M_.resize(Dimension_q_+Dimension_constraint_+1,Dimension_q_+Dimension_constraint_+1);			M_.setZero();
	Handle_M_I_.resize(Dimension_q_+Dimension_constraint_+1,Dimension_q_+Dimension_constraint_+1);	Handle_M_I_.setZero();
	b_.resize(Dimension_q_+Dimension_constraint_+1);												b_.setZero();
	I_.resize(Dimension_q_+Dimension_constraint_+1,Dimension_q_+Dimension_constraint_+1); 			I_.setIdentity();

	U_plus_.resize(Dimension_q_+Dimension_constraint_+1);											U_plus_.setZero();
	U_minus_.resize(Dimension_q_+Dimension_constraint_+1);											U_minus_.setZero();
	U_.resize(Dimension_q_+Dimension_constraint_+1);												U_.setZero();
	DU_.resize(Dimension_q_+Dimension_constraint_+1);												DU_.setZero();
	P_Omega_.resize(Dimension_q_+Dimension_constraint_+1);											P_Omega_.setZero();
	Handle_projection.resize(Dimension_q_+Dimension_constraint_+1);									Handle_projection.setZero();
	CE_QP_.resize(Dimension_constraint_,Dimension_q_);												CE_QP_.setZero();
	ce0_QP_.resize(Dimension_constraint_);															ce0_QP_.setZero();
	X_QP_.resize(Dimension_q_);																		X_QP_.setZero();

	Theta_plus_.resize(Dimension_q_);			Theta_plus_.setZero();
	Theta_minus_.resize(Dimension_q_);			Theta_minus_.setZero();
	DGamma_.resize(Dimension_q);				DGamma_.setZero();
	Gamma_=2;

	Const_Upper_.resize(N_robots_);
	Const_Lower_.resize(N_robots_);

	restart_the_robots();

	if (Solver_type_==Numerical)
	{
		if (Solver_N_Type==Nlopt)
		{
			G_QP_.resize(Dimension_q_,Dimension_q_);								G_QP_.setIdentity();
			g0_QP_.resize(Dimension_q_);											g0_QP_.setZero();


			data_object.J.resize(Dimension_constraint_,Dimension_q_);				data_object.J.setZero();
			data_object.X.resize(Dimension_constraint_);							data_object.X.setZero();
			lb_.resize(Dimension_q_,0);
			ub_.resize(Dimension_q_,0);
			opt= new nlopt::opt(nlopt::LN_BOBYQA, Dimension_q_);
		}
		else if (Solver_N_Type==CVXgen1)
		{
			G_QP_.resize(Dimension_q_,Dimension_q_);								G_QP_.setIdentity();
			g0_QP_.resize(Dimension_q_);											g0_QP_.setZero();


			set_defaults();
			setup_indexing();

		}

	}
	filter= new SGF::SavitzkyGolayFilter(Dimension_q_,order, winlen, sample_time);
	inp.resize(Dimension_q_);
	outp.resize(Dimension_q_);
	cout<<"Dimension_constraint "<<Dimension_constraint_<<endl;
	cout<<"Dimension q "<<Dimension_q_<<endl;
}
void qp_ik_solver::set_jacobian(int index,MatrixXd Jacobian)
{
	/* Setting the current jacobian
	 * 								*/

	if ((Robots_[index].Jacobian.cols()!=Jacobian.cols())||(Robots_[index].Jacobian.rows()!=Jacobian.rows()))
	{
		cout<<"Jacobian of "<<index<<"th robot is wrong."<<endl;
		cout<<"The input Jacobian dimension "<<Jacobian.rows()<<" * "<<Jacobian.cols()<<endl;
		cout<<"The robot Jacobian dimension "<<Robots_[index].Jacobian.rows()<<" * "<<Robots_[index].Jacobian.cols()<<endl;
		ERROR();
	}
	if (Robots_[index].jacobian_is_set==true)
	{
		cout<<"Jacobian of "<<index<<"th robot is already being set."<<endl;
		ERROR();
	}
	Robots_[index].Jacobian=Jacobian;
	Robots_[index].jacobian_is_set=true;

}

void qp_ik_solver::set_jacobian_links(int index,Jacobian_S Jacobian)
{
	/* Setting the current jacobian of all the links!
	 * For obstacle avoidance!
	 * 								*/
	Robots_[index].Jacobian_R=Jacobian;

	/*	for	(int i=0;i<7;i++)
	{
		cout<<"Jacobian  of "<<index<<" robot. No "<< i<<"of jacobian"<<endl<<Robots_[index].Jacobian_R.Jacobian[i]<<endl;
		cout<<"The full Jacobian  of "<<index<<" robot. No "<< i<<"of jacobian"<<endl<<Robots_[index].Jacobian_R.Jacobian_7[i]<<endl;
	}*/

}




void qp_ik_solver::set_state(int index,VectorXd q,VectorXd Dq)
{
	/* Setting the current state of the  index th robot
	 * q is the position of the joints
	 * Dq is the velocity of the joints
	 * 								*/
	if ((Robots_[index].q.rows()!=q.rows())||(Robots_[index].Dq.rows()!=Dq.rows()))
	{
		cout<<"The state dimension of "<<index<<"th robot is wrong."<<endl;
		cout<<"The dimension of q and Dq are "<<q.rows()<<" * "<<Dq.rows()<<endl;
		cout<<"The dimension of robot is "<<Robots_[index].N_links<<endl;
		ERROR();
	}
	if (Robots_[index].state_is_set==true)
	{
		cout<<"States of "<<index<<"th robot is already being set."<<endl;
		ERROR();
	}

	Robots_[index].q=q;
	Robots_[index].Dq=Dq;
	Robots_[index].state_is_set=true;

}
VectorXd qp_ik_solver::Omega_projector(VectorXd Input,VectorXd Upper,VectorXd Lower){

	/*	Simply saturate the input vector between
	 *  the Upper and the Lower bounds
	 * 											*/

	if ((Input.rows()!=Upper.rows())||(Input.rows()!=Lower.rows()))
	{
		cout<<"Problem with Omega_projector."<<endl;
		cout<<"Input dimension "<<Input.rows()<<" Upper dimension "<<Upper.rows() <<" Lower dimension "<<Lower.rows()<<endl;
		ERROR();
	}

	VectorXd output(Input.rows());
	for (int i=0;i<Input.rows();i++)
	{
		output(i)=std::min(std::max(Input(i), Lower(i)), Upper(i));
	}
	return Input;

}
void qp_ik_solver::set_desired(int index,VectorXd Desired_end){

	/* Setting the current set_desired
	 * 								*/

	if (Robots_[index].Desired_end.rows()!=Desired_end.rows())
	{
		cout<<"Desired_end of "<<index<<"th robot is wrong."<<endl;
		cout<<"The input Desired_end dimension "<<Desired_end.rows()<<endl;
		cout<<"The robot Desired_end dimension "<<Robots_[index].Desired_end.rows()<<endl;
		ERROR();
	}
	if (Robots_[index].desired_position_is_set==true)
	{
		cout<<"Desired_position_is_set of "<<index<<"th robot is already being set."<<endl;
		ERROR();
	}
	Robots_[index].Desired_end = Desired_end;
	Robots_[index].desired_position_is_set=true;
}


void qp_ik_solver::get_gamma(double& gamma){
	gamma = Gamma_;
}

void qp_ik_solver::Solve()
{
	/* Solving the IK problem
	 * 								*/
	for (int i=0;i<N_robots_;i++)
	{
		if ((Robots_[i].desired_position_is_set==false)||(Robots_[i].jacobian_is_set==false)||(Robots_[i].state_is_set==false))
		{
			cout<<"Can not solve the IK as parameters of "<<i<<"th robot are wrong."<<endl;
			cout<<"The desired_position_is_set "<<Robots_[i].desired_position_is_set<<endl;
			cout<<"The jacobian_is_set "<<Robots_[i].jacobian_is_set<<endl;
			cout<<"The state_is_set "<<Robots_[i].state_is_set<<endl;
			ERROR();

		}
	}


    if (considerCollision){
        //clock_t t;
       // t = clock();
        Construct_collision_boundaries();
       // t = clock() - t;
      //  cout << "Boundary Construction Time: " << ((float)t)/CLOCKS_PER_SEC << endl;
//        myfile_time << ((float)t)/CLOCKS_PER_SEC << endl;
    }


	if (Solver_type_==Dynamical)
	{
		VectorXd U_Handle_(Dimension_q_);
		t1 = clock();
		Construct_boundaries_vel();
		Construct_vel();

		Handle_M_I_=I_+M_.transpose();

		int counter=0;
		duration=0;
		//while (((float)duration)/CLOCKS_PER_SEC<dt_/2)
		while ((U_Handle_-U_.block(0,0,Dimension_q_,1)).norm()>0.0001)
		{
			U_Handle_=U_.block(0,0,Dimension_q_,1);
			Handle_projection=U_-(M_*U_+b_);
			for (int i=0;i<Handle_projection.rows();i++)
			{
				Handle_projection(i)=std::min(std::max(Handle_projection(i), U_minus_(i)), U_plus_(i))-U_(i);
			}
			U_.noalias() +=Handle_M_I_*(Handle_projection)*(dt_);
			counter=counter+1;
			duration = clock() - t1;

		}
		//cout<<"(U_Handle_-U_.block(0,0,Dimension_q_,1).norm()) "<<(U_Handle_-U_.block(0,0,Dimension_q_,1)).norm()<<" "<<counter<<endl;
		if (save_the_performace){

			int counter_N=0;
			int counter_q=0;
			for (int i=0;i<N_robots_;i++)
			{
				CE_QP_.block(counter_N,counter_q,Robots_[i].N_constraints,Robots_[i].N_links)=Robots_[i].Jacobian;
				ce0_QP_.block(counter_N,0,Robots_[i].N_constraints,1)=Robots_[i].Desired_end;
				counter_q=counter_q+Robots_[i].N_links;
				counter_N=counter_N+Robots_[i].N_constraints;
			}
			myfile<<((float)duration)/CLOCKS_PER_SEC<<" "<<(CE_QP_*X_QP_-ce0_QP_).norm();
			for (int i=0;i<Dimension_q_;i++)
			{
				X_QP_(i)=U_(i);
				myfile<<" "<<X_QP_(i);
			}
			myfile<<" "<<endl;
		}


		restart_the_robots();

	}
	else if (Solver_type_==Numerical)
	{
		int counter_N=0;
		int counter_q=0;
		for (int i=0;i<N_robots_;i++)
		{
			CE_QP_.block(counter_N,counter_q,Robots_[i].N_constraints,Robots_[i].N_links)=Robots_[i].Jacobian;
			ce0_QP_.block(counter_N,0,Robots_[i].N_constraints,1)=Robots_[i].Desired_end;
			counter_q=counter_q+Robots_[i].N_links;
			counter_N=counter_N+Robots_[i].N_constraints;
		}

		if (Solver_N_Type==Nlopt)
		{
			t1 = clock();
			int counter_q=0;
			for (int i=0;i<N_robots_;i++)
			{
				for (int j=0;j<Robots_[i].N_links;j++)
				{
					lb_[counter_q]=std::max(mu_p_*(eta_p_*Robots_[i].L_q(j)-Robots_[i].q(j)),Robots_[i].L_Dq(j));
					ub_[counter_q]=std::min(mu_p_*(eta_p_*Robots_[i].U_q(j)-Robots_[i].q(j)),Robots_[i].U_Dq(j));
					counter_q++;
				}
			}
			opt->set_lower_bounds(lb_);
			opt->set_upper_bounds(ub_);

			data_object.J=CE_QP_;
			data_object.desired=ce0_QP_;
			data_object.X.resize(Dimension_q_);
			opt->set_min_objective(myvfunc, &data_object);

			opt->set_maxtime(1);
			std::vector<double>  x;x.resize(Dimension_q_,0);
			double result;
			nlopt::result nlopt_result=opt->optimize(x,result);
			for (int i=0;i<Dimension_q_;i++)
			{
				X_QP_(i)=x[i];
			}
			duration = clock() - t1;
			opt->remove_equality_constraints();
			opt->remove_inequality_constraints();
			myfile<<((float)duration)/CLOCKS_PER_SEC<<" "<<(CE_QP_*X_QP_-ce0_QP_).norm();
			for (int i=0;i<Dimension_q_;i++)
			{
				myfile<<" "<<X_QP_(i);
			}
			myfile<<" "<<endl;
		}
		/*
				cout<<"nlopt::result "<<nlopt_result<<" result "<<result<<endl;
				cout<<"CE_QP_ "<<endl<<CE_QP_<<endl;
				cout<<"ce0_QP_ "<<endl<<ce0_QP_<<endl;
				cout<<"result "<<endl<<CE_QP_*X_QP_<<endl;
				cout<<"X_QP_ "<<endl<<X_QP_<<endl;

				ERROR();*/
		else if (Solver_N_Type==CVXgen1)
		{

			t1 = clock();
			load_default_data();
			settings.verbose =0;
			int num_iters = solve();
			for (int i=0;i<Dimension_q_;i++)
			{
				inp(i)=vars.q[i];
			}
			ret_code = filter->AddData(inp);
			ret_code = filter->GetOutput(0, outp);
			for (int i=0;i<Dimension_q_;i++)
			{
				X_QP_(i)=outp(i);
			}
			duration = clock() - t1;
			myfile<<((float)duration)/CLOCKS_PER_SEC<<" "<<(CE_QP_*X_QP_-ce0_QP_).norm();
			for (int i=0;i<Dimension_q_;i++)
			{
				myfile<<" "<<X_QP_(i);
			}
			myfile<<" "<<endl;
		}
		//printf ("It took me %d clicks (%f seconds).\n",duration,((float)duration)/CLOCKS_PER_SEC);
		restart_the_robots();
	}
}
void qp_ik_solver::get_state(int index, VectorXd &Dq)
{

	/* getting the desired joint velocity of index th robot
	 * 														*/

	int counter=0;

	for (int i=0;i<index;i++)
	{
		counter=counter+Robots_[i].N_links;
	}
	if (Solver_type_==Numerical)
	{
		Dq.resize(Robots_[index].N_links);
		Dq=X_QP_.block(counter,0,Robots_[index].N_links,1);
	}
	else if (Solver_type_==Dynamical)
	{
		Dq.resize(Robots_[index].N_links);
		Dq=U_.block(counter,0,Robots_[index].N_links,1);


	}

}
inline void qp_ik_solver::Check_feasibility(VectorXd U)
{
	for (int i=0;i<U_plus_.rows();i++)
	{
		if ((U_plus_(i)<U_(i))||(U_minus_(i)>U_(i)))
		{
			cout<<"Boundary is fucked "<<endl;
			cout<<"U_plus_ "<<endl;cout << U_plus_.format(CleanFmt) << endl;
			cout<<"U_minus_ "<<endl;cout << U_minus_.format(CleanFmt) << endl;
			cout<<"U_ "<<endl;cout << U_.format(CleanFmt) << endl;
			ERROR();

		}
	}
}
inline void qp_ik_solver::Construct_boundaries_vel()
{
	int counter=0;
	Const_Upper_.setConstant(1000000);
	Const_Lower_.setConstant(1000000);
	double handle_1;
	double handle_2;
	for (int i=0;i<N_robots_;i++)
	{
		for (int j=0;j<Robots_[i].N_links;j++)
		{
			Theta_minus_(counter)=std::max(mu_p_*(eta_p_*Robots_[i].L_q(j)-Robots_[i].q(j)),Robots_[i].L_Dq(j));
			Theta_plus_(counter)=std::min(mu_p_*(eta_p_*Robots_[i].U_q(j)-Robots_[i].q(j)),Robots_[i].U_Dq(j));
			//			handle_1=std::min((eta_p_*Robots_[i].U_q(j)-Robots_[i].q(j)),(Robots_[i].U_Dq(j)-Robots_[i].Dq(j)));
			handle_1=(eta_p_*Robots_[i].U_q(j)-Robots_[i].q(j));
			if (handle_1<Const_Upper_(i))
			{
				Const_Upper_(i)=handle_1;

			}
			//			handle_2=std::min((Robots_[i].q(j)-eta_p_*Robots_[i].L_q(j)),(Robots_[i].Dq(j)-Robots_[i].L_Dq(j)));
			handle_2=(Robots_[i].q(j)-eta_p_*Robots_[i].L_q(j));
			if (handle_2<Const_Lower_(i))
			{
				Const_Lower_(i)=handle_2;

			}
			counter=counter+1;
		}
	}

	counter=0;

	if (Super_constraint_==true)
	{
		for (int i=0;i<N_robots_;i++)
		{
			for (int j=0;j<Robots_[i].N_constraints;j++)
			{
				U_plus_(counter+Dimension_q_)=100000/(1+10000*exp(-10*Const_Upper_(i)));
				U_minus_(counter+Dimension_q_)=-100000/(1+10000*exp(-10*Const_Lower_(i)));
				counter=counter+1;
			}
			//		cout<<"Robot "<<i<<" Const_Upper_ "<<100000/(1+10000*exp(-10*Const_Upper_(i)))<<" Const_Lower_ "<<100000/(1+10000*exp(-10*Const_Lower_(i)))<<endl;
		}
	}
	else
	{
		/*		for (int i=0;i<N_robots_;i++)
		{
			cout<<"Robot "<<i<<" Const_Upper_ "<<100000/(1+10000*exp(-10*Const_Upper_(i)))<<" Const_Lower_ "<<100000/(1+10000*exp(-10*Const_Lower_(i)))<<endl;
		}*/
		U_plus_.setConstant(100000);
		U_minus_.setConstant(-100000);
	}


	U_plus_.block(0,0,Dimension_q_,1)=Theta_plus_;
	U_minus_.block(0,0,Dimension_q_,1)=Theta_minus_;
	U_minus_(Dimension_constraint_+Dimension_q_)=0;



}
inline void qp_ik_solver::Construct_vel()
{
	int counter_N=0;
	int counter_q=0;
	//U_.setZero();
	for (int i=0;i<N_robots_;i++)
	{
		W_.block(counter_q,counter_q,Robots_[i].N_links,Robots_[i].N_links)=Robots_[i].W;
		J_.block(counter_N,counter_q,Robots_[i].N_constraints,Robots_[i].N_links)=Robots_[i].Jacobian;
		b_.block(Dimension_q_+counter_N,0,Robots_[i].N_constraints,1)=-Robots_[i].Desired_end;
		//	U_.block(counter_q,0,Robots_[i].N_links,1)=Robots_[i].Dq;
		counter_q=counter_q+Robots_[i].N_links;
		counter_N=counter_N+Robots_[i].N_constraints;

	}

	M_.block(0,0,Dimension_q_,Dimension_q_)=W_;
	M_.block(0,Dimension_q_,Dimension_q_,Dimension_constraint_)=-J_.transpose();
	M_.block(Dimension_q_,0,Dimension_constraint_,Dimension_q_)= J_;

	M_.block(Dimension_constraint_+Dimension_q_,0,1,Dimension_q_)= DGamma_.transpose();
	M_.block(0,Dimension_constraint_+Dimension_q_,Dimension_q_,1)=-DGamma_;
    b_(Dimension_constraint_+Dimension_q_)=lambda*log(Gamma_-1);

	/*	cout<<"W_"<<endl;
	cout << W_.format(CleanFmt) << endl;
	cout<<"J_"<<endl;
	cout << J_.format(CleanFmt) << endl;*/
	/*
	cout<<"M_"<<endl;
	cout << M_.format(CleanFmt) << endl;
	cout<<"b_"<<endl;
	cout << b_.format(CleanFmt) << endl;
	 */


}

inline void qp_ik_solver::Construct_collision_boundaries()
{
    /*Fill DGamma_ and Gamma_
     * Gamma_ is a double
     * DGamma_ is the derivative of Gamma and its dimension is Dimension_q
     *  */

    // Construct input 36D feature vector f(q) and J(f(q))
    VectorXd f_q;MatrixXd J_fq;
    unsigned int collision_joints = 6,  n_fq = 3, nDoF= 7;
    f_q.resize(collision_joints*n_fq*N_robots_);
    J_fq.resize(collision_joints*n_fq*N_robots_, nDoF);

    int block_idx = 0;
    for (unsigned int index=0;index<N_robots_;index++)
        for	(int j=0;j<collision_joints;j++){
            f_q.block(block_idx,0,n_fq,1)     = Robots_[index].Jacobian_R.Link_pos[j];
            J_fq.block(block_idx,0,n_fq,nDoF) = Robots_[index].Jacobian_R.Jacobian_7[j];
            block_idx = block_idx + n_fq;
        }

    // Compute Gamma(f(q)) and DGamma(f(q)) (36D)
    MatrixXd Gamma_der; Gamma_der.resize(collision_joints*n_fq,1);
    Gamma_    = svmBoundary_.calculateGamma(f_q);
    Gamma_der = svmBoundary_.calculateGammaDerivative(f_q);

    // Project DGamma(f(q)) (36D) -> DGamma(q) (14D)
    MatrixXd Gamma_derxJ_fq; Gamma_derxJ_fq.resize(1,nDoF);

    for (unsigned int index=0;index<N_robots_;index++){
        Gamma_derxJ_fq = Gamma_der.block(index*collision_joints*n_fq,0,collision_joints*n_fq,1).transpose()*J_fq.block(index*collision_joints*n_fq,0,collision_joints*n_fq,nDoF);
        DGamma_.block(index*nDoF,0,nDoF,1) = Gamma_derxJ_fq.transpose().normalized();
    }


}
inline void qp_ik_solver::restart_the_robots()
{
	for(int i=0;i<N_robots_;i++)
	{
		Robots_[i].jacobian_is_set=false;
		Robots_[i].desired_position_is_set=false;
		Robots_[i].state_is_set=false;
	}
}
void qp_ik_solver::ERROR()
{
	while(ros::ok())
	{

	}
}
void qp_ik_solver::Print_Robot(S_Robot_IK Robot)
{
	cout<<"Printing of "<<Robot.index<<"th robot."<<endl;
	cout<<"N_links "<<Robot.N_links<<endl;
	cout<<"U_p: "<<endl;cout<<Robot.U_q<<endl;
	cout<<"L_p: "<<endl;cout<<Robot.L_q<<endl;
	cout<<"U_Dp:"<<endl;cout<<Robot.U_Dq<<endl;
	cout<<"L_Dp:"<<endl;cout<<Robot.L_Dq<<endl;
	cout<<"q:"<<endl;cout<<Robot.q<<endl;
	cout<<"W:  	"<<endl;cout<<Robot.W<<endl;
	cout<<"U_DDp: "<<endl;cout<<Robot.U_DDq<<endl;
	cout<<"U_DDp: "<<endl;cout<<Robot.U_DDq<<endl;
}
void qp_ik_solver::load_default_data() {

	for (int i = 0; i < Dimension_constraint_; i++)
		for (int j = 0; j < Dimension_q_; j++)
			params.J[i+j*Dimension_constraint_] =CE_QP_(i,j);

	for (int i=0;i<ce0_QP_.rows();i++)
		params.Xi[i]=ce0_QP_(i);

	int counter=0;
	for (int i=0;i<N_robots_;i++)
	{
		for (int j=0;j<Robots_[i].N_links;j++)
		{
			params.L_min[counter]=std::max(mu_p_*(eta_p_*Robots_[i].L_q(j)-Robots_[i].q(j)),Robots_[i].L_Dq(j));
			params.L_max[counter]=std::min(mu_p_*(eta_p_*Robots_[i].U_q(j)-Robots_[i].q(j)),Robots_[i].U_Dq(j));
			counter++;
		}
	}

	for (int j = 0; j < Dimension_q_; j++)
	{
		params.A[j]=-DGamma_(j);
	}
	if (Gamma_<1.2)
	{
		params.b[0]=lambda*log(0.2);
	}
	else
	{
	    params.b[0]=lambda*log(Gamma_-1);
	}


}
