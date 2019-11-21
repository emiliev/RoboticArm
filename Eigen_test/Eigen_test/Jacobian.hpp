//
//  Jacobian.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 18.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef Jacobian_hpp
#define Jacobian_hpp

#include <stdio.h>
#include "MatrixFactory.hpp"
#include "SharedTypes.h"
#include "RobotType.h"

class Jacobian
{
/************************************************************************/
/* Jacobian holder                                                      */
/************************************************************************/
    Eigen::MatrixXf  _jacobian;
/************************************************************************/
/* Matrix and vector help functions                                     */
/************************************************************************/
    MatrixFactory* mtxf;

    Jacobian();

    static Jacobian* _instance;

public:
    
/************************************************************************/
/* Set and get functions                                                */
/************************************************************************/
    void setJacobianConfiguration(unsigned int row , unsigned int col);
    Eigen::MatrixXf & getJacobian();
/************************************************************************/
/* One column of J calculation function                                 */
/************************************************************************/
    void calculateColumnOfJacobian_New(HomMatrixHolder& hom_matrix_handler,
                                       unsigned int ind ,
                                       JointT jt,
                                       Eigen::Matrix4f& full);

    void calculateJacobian(HomMatrixHolder& hom_matrix_handler ,
                           JointHandler& jhandler,
                           Eigen::Matrix4f& full);

    Eigen::MatrixXf  psevdoInverse();
    
    static Jacobian * getInstance();
};



#endif /* Jacobian_hpp */
