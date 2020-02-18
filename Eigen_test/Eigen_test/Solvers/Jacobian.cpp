//
//  Jacobian.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 18.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#include "Jacobian.hpp"
#include <iostream>
#include <Eigen/SVD>

Jacobian::Jacobian() : mtxf(MatrixFactory::getInstance()){
}

void Jacobian::setJacobianConfiguration( unsigned int row , unsigned int col ) {
    //We don't know configuration
    _jacobian.resize(row , col);
}

Eigen::MatrixXf& Jacobian::getJacobian() {
    //Get calculated Jacobian
    return _jacobian;
}

// Transformation matrix representation
//
//|r11 r12 r13 d14|
//|r21 r22 r23 d24|
//|r31 r32 r33 d34|
//|  0   0   0   1|
//

/************************************************************************/
/* Main routine for J calculation                                       */
/************************************************************************/
void Jacobian::calculateJacobian( HomMatrixHolder& hom_matrix_handler , JointHandler& jhandler , Eigen::Matrix4f& full) {
    unsigned int col_num = jhandler.size();

    if (!col_num) {
        //Zero size not allowed
        return;
    }

    unsigned int row_num = NUMBEROFSETPARAMETERS;
    unsigned int size = hom_matrix_handler.size();
    //Set J confiruration
    setJacobianConfiguration(row_num , col_num);
    
    std::vector<Eigen::Matrix4f> trans_matrix_holder;
    for (int index = 0; index < size + 1; ++index) {
        trans_matrix_holder.push_back(Eigen::Matrix4f::Identity());
    }

    for (int index = 1; index < size + 1; ++index) {
        trans_matrix_holder[index] = trans_matrix_holder[index - 1] * hom_matrix_handler[index - 1];
    }

    for (unsigned int i = 1 ; i < col_num + 1 ; ++i) {
        calculateColumnOfJacobian_New(trans_matrix_holder[i - 1],DHINDEX(i),jhandler[DHINDEX(i)].getJointType(),full);
    }
}

Jacobian* Jacobian::_instance = NULL;

Jacobian* Jacobian::getInstance() {
    if (!_instance)
        _instance = new Jacobian();

    return _instance;
}

Eigen::MatrixXf Jacobian::psevdoInverse() {
    Eigen::MatrixXf inv;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
    svd.compute(_jacobian , Eigen::ComputeThinU | Eigen::ComputeThinV);
//    svd.pinv(inv);
    double epsilon = std::numeric_limits<double>::epsilon();
    double tolerance = epsilon * std::max(_jacobian.cols(), _jacobian.rows()) *svd.singularValues().array().abs()(0);
    inv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    return inv;
}

void Jacobian::calculateColumnOfJacobian_New(Eigen::Matrix4f& hom_matrix_handler, unsigned int ind, JointT jt, Eigen::Matrix4f& fullm) {
    Eigen::Vector3f z0(0.0f , 0.0f , 1.0f);
    Eigen::Vector3f zi;
    Eigen::Matrix4f transf_matrix = hom_matrix_handler;
    Eigen::Matrix3f rot_m;

    Eigen::Vector3f p_end_effector;
    Eigen::Vector3f pi;

    //Position of end effector
    p_end_effector<< fullm(0,3), fullm(1,3), fullm(2,3);
    rot_m = transf_matrix.block(0,0,3,3);

    //
    //  Zi-1
    //
    zi = rot_m * z0;
    pi << transf_matrix(0,3) , transf_matrix(1,3) , transf_matrix(2,3);

    //
    //  (Pe - Pi-1)
    //
    Eigen::Vector3f delta_vec = p_end_effector - pi;

    //
    //  Zi x (Pe - Pi-1)
    //
    Eigen::Vector3f d_rev = zi.cross(delta_vec);

    //We should get type of joint and go further
    switch(jt) {
    case PRISMATIC:
        //For prismatic joint everything is simple :
        //        | z | <--- calculated vector z
        // Cind = |   |
        //        | 0 | <--- zero vector3f
        _jacobian(0,ind) = zi(0);
        _jacobian(1,ind) = zi(1);
        _jacobian(2,ind) = zi(2);
        _jacobian(3,ind) = 0.0f;
        _jacobian(4,ind) = 0.0f;
        _jacobian(5,ind) = 0.0f;
        break;
    case REVOLUTE:
        //For revolute joint everything is harder :
        //        | z * d | <--- calculated vector z * vector d
        // Cind = |       |
        //        |   z   | <--- calculated vector z
        _jacobian(0,ind) = d_rev(0);
        _jacobian(1,ind) = d_rev(1);
        _jacobian(2,ind) = d_rev(2);
        _jacobian(3,ind) = zi(0);
        _jacobian(4,ind) = zi(1);
        _jacobian(5,ind) = zi(2);
        break;
    default:
        break;
    }
}
