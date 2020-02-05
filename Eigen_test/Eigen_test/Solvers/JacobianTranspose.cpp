//
//  JacobianTranspose.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 19.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#include "JacobianTranspose.hpp"
#include "Jacobian.hpp"
#include "Robot.hpp"
#include <iostream>

JacobianTranspose::JacobianTranspose(std::shared_ptr<Robot> robot):
        mtxinstance(MatrixFactory::getInstance()),
        _robot(robot) {
    _desired_position = Eigen::VectorXf(6);
    current_position.resize(6);
}

void JacobianTranspose::setAdditionalParameter(float& add_in ) {
    lamda_coefficent = add_in;
}

void JacobianTranspose::setDesiredPosistion(Eigen::VectorXf&  desired_position) {
    this->_desired_position = desired_position;
}

Eigen::VectorXf JacobianTranspose::calculateData() {
    //Result vector
    Eigen::VectorXf delta_theta(_robot->giveMeMatrixHolder().size());
    //Vector for delta moving in space
    Eigen::VectorXf delta_translation(6);

    Eigen::VectorXf _temp = _desired_position;

    Jacobian* jac = Jacobian::getInstance();
    //TODO
    //should be a parameter of function
    //Desired accuracy
    //TODO: MAKR:- add desired accruacy
    float epsilon = 0.1f;

    for (;;) {
        jac->calculateJacobian(_robot->giveMeMatrixHolder(),
                               _robot->giveMeJoints(),
                               _robot->giveMeFullHM());
        //calculation delta
        current_position << _robot->giveMeFullHM()(0,3) ,    //X
                            _robot->giveMeFullHM()(1,3) ,    //Y
                            _robot->giveMeFullHM()(2,3) ,    //Y
                            0.0f,
                            0.0f,
                            0.0f;
                            
        delta_translation = _desired_position - current_position;
        //compare delta with desired accuracy
        float n = delta_translation.norm();
        if (n < epsilon) {
            //Done
            break;
        }

        //Lets calculate lambda
        //TODO optimize it

        Eigen::MatrixXf _jac_tr =  jac->getJacobian().transpose();
        
        Eigen::VectorXf upper = jac->getJacobian() * _jac_tr * delta_translation;
        Eigen::VectorXf double_upper = upper;
        float one = delta_translation.dot(upper);
        float down  = upper.dot(double_upper);

        lamda_coefficent = one/down;


        delta_theta = lamda_coefficent * jac->getJacobian().transpose() * delta_translation;
        updateJoints(delta_theta);
    }

    /// Send notification
    _robot->notifyUpdatedJoints();
    return _temp;
}

void JacobianTranspose::updateJoints(Eigen::VectorXf & delta_theta) {
    JointHandler& joints = _robot->giveMeJoints();

    for (int i = 0 ; i < delta_theta.size() ; i++) {
        //new var = old var + delta var
        float old = joints[i].giveMeVariableParametr();
        float delta = delta_theta[i];
        float res = delta + old;
        _robot->setJointVariable(i,res);
    }
    _robot->caclulateFullTransormationMatrix();
}



