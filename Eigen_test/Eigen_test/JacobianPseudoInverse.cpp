//
//  JacobianPseudoInverse.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 19.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#include "JacobianPseudoInverse.hpp"
#include "Jacobian.hpp"
#include <iostream>
#include <Eigen/StdVector>

JacobianPseudoInverse::JacobianPseudoInverse(Robot& robot):
    mtxinstance(MatrixFactory::getInstance()),
//    _desired_position(Eigen::VectorXf(6)),
    _robot(robot) {
        _desired_position = Eigen::VectorXf(6);
//        re
//        _desired_position = Vector
        current_position.resize(6);
}

Eigen::VectorXf JacobianPseudoInverse::calculateData(Eigen::VectorXf& desired_position) {
    //Result vector
    Eigen::VectorXf delta_theta(_robot.giveMeMatrixHolder().size());
    //Vector for delta moving in space
    Eigen::VectorXf delta_translation(6);

    Eigen::VectorXf _temp = _desired_position;

    Jacobian* jac = Jacobian::getInstance();
    //MARK: - TODO
    //should be a parameter of function
    //Desired accuracy
    float epsilon = 0.1f;
    
    for (;;)
    {
        jac->calculateJacobian(_robot.giveMeMatrixHolder(),_robot.giveMeJoints(),_robot.giveMeFullHM());
        //calculation delta
        current_position << _robot.giveMeFullHM()(0,3) ,    //X
                            _robot.giveMeFullHM()(1,3) ,    //Y
                            _robot.giveMeFullHM()(2,3) ,    //Z
                            0.0f,                           //Orientation
                            0.0f,                           //Orientation
                            0.0f;                           //Orientation

//        std::cout<<"Current position"<<std::endl<<current_position<<std::endl;
//        std::cout<<"Desired position"<<std::endl<<_desired_position<<std::endl;
        delta_translation = _desired_position - current_position;
//        std::cout<<"Delta position"<<std::endl<<delta_translation<<std::endl;

        //compare delta with desired accuracy
        float n = delta_translation.norm();
        if (n < epsilon) {
            //Done
            break;
        }
        //Algo
        delta_theta = jac->psevdoInverse() * delta_translation;
        
//        std::cout<<"Delta theta"<<std::endl<<delta_theta<<std::endl;
        updateJoints(delta_theta);
    }

    return _temp;
}

void JacobianPseudoInverse::updateJoints(Eigen::VectorXf& delta_theta ) {
    JointHandler & _j = _robot.giveMeJoints();

    for (int i = 0 ; i < delta_theta.size() ; i++) {
        //TODO ADD CONSTRAINTS
        //new var = old var + delta var
        float res = delta_theta[i] + _j[i].giveMeVariableParametr();
        _robot.setJointVariable(i,res);
    }
}
