# QP_IK_solver
[![Build Status](https://travis-ci.org/sinamr66/QP_IK_solver.svg?branch=master)](https://travis-ci.org/sinamr66/QP_IK_solver)

This repository includes the packages and instructions to solve the centralized inverse kinematic problem, formulated as a quadratic program (QP) and solved in real-time. You can find the paper here:


and the corresponding video here: 




#  Dependencies
Eigen http://eigen.tuxfamily.org/index.php?title=Main_Page

Svm_grad https://github.com/nbfigueroa/SVMGrad.git

Sg_differentiation https://github.com/epfl-lasa/sg_differentiation.git

Nlopt http://ab-initio.mit.edu/wiki/index.php/NLopt

CVXGEN http://cvxgen.com/docs/index.html **(Due to lisence, I needed to remove CVxgen, please follow the steps in the mentioned webpage to get the CVxgen files.)**

# Features:

- Handling the kinematic constraints of the robots.
- Inequity constraint (We used it for considering the self-collision avoidance (SCA) constraints).
- This package provides three options for solving the inverse kinematic:
  - LVI-based primal-dual Dynamical system solution: http://ieeexplore.ieee.org/abstract/document/1470152/
  - Nlopt
  - CVXgen
  
# How to run
### Initialization:
1-Initialize the IK solver:
  1-1 If You don't want to use the SCA constraint
```
Initialize(Number of robots ,Time sample ,Solver_type={Numerical=0,Dynamical},Solver_level={Velocity_level=0,Acceleration_level},Super_constraint={True, False})
```
  1-2 If You want to use the SCA constraint
```
Initialize(Number of robots ,Time sample ,Solver_type={Numerical=0,Dynamical},Solver_level={Velocity_level=0,Acceleration_level},Super_constraint={True, False},svm_filename)
```
The Dynamical solver uses the LVI-based primal-dual Dynamical system solver and the default Numerical solver is CVXGEN. If you want to change it to Nlopt you need to modify  [qp_ik_solver.h](https://github.com/sinamr66/QP_IK_solver/blob/master/include/qp_ik_solver.h). svm_filename is a path to the learned SVR collision configurations. 
2- Initialize each robot:
```
Initialize_robot(index of the robot,Number of links,Number of end-effector constraints,W,Upper bound of joints' position, Lower bound of joints' position,Upper bound of joints' velocity,Lower bound of joints' velocity, Upper bound of joints' acceleration, Lower bound of joints' acceleration)
``` 
3- Finalize initialization 
```
Finalize_Initialization()
``` 
### In the loop:
4- In the loop:
```
set_jacobian(index of the robot,Jacobian of the end-effector)
set_jacobian_links(index of the robot,Jacobian and Cartesian position of all the links)
set_state(index of the rooot,current joint position,current joint velocity);
set_desired(index of the robot,Desired end-effector state);
Solve();
get_state(index of the robot, desired joint velocity);
```
## Copyright
Please cite these papers if you are using this toolbox:
@article{mirrazavi2018unified,
  title={A unified framework for coordinated multi-arm motion planning},
  author={Mirrazavi Salehian, Seyed Sina and Figueroa, Nadia and Billard, Aude},
  journal={The International Journal of Robotics Research},
  pages={0278364918765952},
  publisher={SAGE Publications Sage UK: London, England}
}

For more information contact Sina Mirrazavi.
