//
//  DumpbedLeastSquares.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#include "DumpbedLeastSquares.hpp"
#include "Jacobian.hpp"
#include <iostream>

DumpedLeastSquares::DumpedLeastSquares(Eigen::VectorXf& desired_position , Robot& robot ):
                                                                                                mtxinstance(MatrixFactory::getInstance()),
                                                                                                _desired_position(desired_position),
                                                                                                _robot(robot) {
    current_position.resize(6);
}

Eigen::VectorXf DumpedLeastSquares::calculateData() {
    //Result vector
    Eigen::VectorXf delta_theta(_robot.giveMeMatrixHolder().size());
    //Vector for delta moving in space
    Eigen::VectorXf delta_translation(6);

    Eigen::VectorXf _temp = _desired_position;

    Jacobian* jac = Jacobian::getInstance();
    //TODO
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

#ifdef JACOBIANDEBUGOUTPUT
        std::cout<<"Current position"<<std::endl<<current_position<<std::endl;
        std::cout<<"Desired position"<<std::endl<<_desired_position<<std::endl;
#endif

        delta_translation = _desired_position - current_position;

#ifdef JACOBIANDEBUGOUTPUT
        std::cout<<"Delta position"<<std::endl<<delta_translation<<std::endl;
#endif
        //compare delta with desired accuracy
        float n = delta_translation.norm();
        if (n < epsilon) {
            //Done
            break;
        }

        //Algorithm
        //TODO optimization needed

        Eigen::MatrixXf one = jac->getJacobian();
        Eigen::MatrixXf two = one * jac->getJacobian().transpose();
        Eigen::MatrixXf id = Eigen::MatrixXf::Identity(two.rows() , two.cols());
        id = (nu * nu) * id;

        Eigen::MatrixXf result = two + id ;
        Eigen::MatrixXf result_out;

        Eigen::JacobiSVD<Eigen::MatrixXf> svd;
        svd.compute(result , Eigen::ComputeThinU | Eigen::ComputeThinV);

/**/
        double epsilon = std::numeric_limits<double>::epsilon();
        double tolerance = epsilon * std::max(result.cols(), result.rows()) *svd.singularValues().array().abs()(0);
        result_out = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
/**/
#ifdef JACOBIANDEBUGOUTPUT
        std::cout<<"Result"<<std::endl<<result_out<<std::endl;
#endif

        result_out = jac->getJacobian().transpose() * result_out;
        
        delta_theta =  result_out * delta_translation;

#ifdef JACOBIANDEBUGOUTPUT
        std::cout<<"Delta theta"<<std::endl<<delta_theta<<std::endl;
#endif
        updateJoints(delta_theta);
    }

    return _temp;
}

void DumpedLeastSquares::updateJoints( Eigen::VectorXf& delta_theta ) {
    JointHandler & _j = _robot.giveMeJoints();
    for (int i = 0 ; i < delta_theta.size() ; i++) {
        //TODO ADD CONSTRAINTS
        //new var = old var + delta var
        float res = delta_theta[i] + _j[i].giveMeVariableParametr();
        _robot.setJointVariable(i,res);
    }
}
