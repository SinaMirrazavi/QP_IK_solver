# QP_IK_solver
[![Build Status](https://travis-ci.org/sinamr66/QP_IK_solver.svg?branch=master)](https://travis-ci.org/sinamr66/QP_IK_solver)

This repository includes the packages and instructions to solve the centralized inverse kinematic problem, formulated as a quadratic program (QP) and solved in real-time. You can find the paper here:


and the corresponding video here: 




#  Dependencies
Eigen http://eigen.tuxfamily.org/index.php?title=Main_Page

Svm_grad https://github.com/nbfigueroa/SVMGrad.git

Sg_differentiation https://github.com/epfl-lasa/sg_differentiation.git

Nlopt http://ab-initio.mit.edu/wiki/index.php/NLopt

CVXGEN http://cvxgen.com/docs/index.html **(In this package, Cvxgen for two 7 DOF robots has already been included. If you want to use this package for other robots, you just need to create new CVxgen files.)**

# Features:

- Handling the kinematic constraints of the robots.
- Inequity constraint (We used it for considering the self-collision avoidance constraints).
- This package provides three options for solving the inverse kinematic:
  - LVI-based primal-dual Dynamical system solution: http://ieeexplore.ieee.org/abstract/document/1470152/
  - Nlopt
  - CVXgen
