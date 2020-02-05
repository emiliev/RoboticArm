//
//  Robot.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef Robot_hpp
#define Robot_hpp

#include <stdio.h>
#include "RobotType.h"
#include "MatrixFactory.hpp"
#include "SharedTypes.h"

class Robot {
/************************************************************************/
/* Original robot configuration                                         */
/************************************************************************/
    dh_table robot_dh;
    unsigned int number_of_var_parameters;
/************************************************************************/
/* Origin aka x0y0z0                                                                     */
/************************************************************************/
    Eigen::Vector3f zero_origin;
/************************************************************************/
/* Joint,Links,Matrix containers                                        */
/************************************************************************/
    JointHandler    jhandle;
    LinkHandler     linkhadle;
    HomMatrixHolder hmtx;
/************************************************************************/
/* Full transformation matrix (from i frame to 0 frame)                 */
/************************************************************************/
    Eigen::Matrix4f        from_i_1_to_i;
/************************************************************************/
/* Matrix factory (not pattern)                                         */
/************************************************************************/
    MatrixFactory * matrix_algo;
public:
    Robot(Eigen::Vector3f& vec);
    bool loadConfig(const dh_table& tbl);
    void setOrigin(Eigen::Vector3f& newOrigin);

    bool rotateJoints(std::vector<float> angles);
    bool rotateJoint(unsigned int ind ,float angle);
    bool translateJoint(unsigned int ind , float displasment);
    bool setJointVariable(unsigned int , float);

    bool printHomogenTransformationMatrix();
    bool caclulateFullTransormationMatrix();

    bool printFullTransformationMatrix();
    bool calculateNumberOfVariableParametrs();
    bool calculateJoint(unsigned int ind);
    void notifyUpdatedJoints();
    /************************************************************************/
    /* Help functions                                                       */
    /************************************************************************/
    Eigen::Matrix4f & giveMeFullHM()
    {
        return from_i_1_to_i;
    }

    Robot* giveMeRobot()
    {
        return this;
    }

    JointHandler& giveMeJoints()
    {
        return jhandle;
    }

    HomMatrixHolder & giveMeMatrixHolder()
    {
        return  hmtx;
    }

    void printConfiguration();

};


#endif /* Robot_hpp */
