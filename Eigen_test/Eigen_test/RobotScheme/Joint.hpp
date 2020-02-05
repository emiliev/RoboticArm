//
//  Joint.hpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef Joint_hpp
#define Joint_hpp

#include <stdio.h>
#include "SharedTypes.h"
#include <string>

class Joint {
    std::string joint_name;

    /************************************************************************/
    /* Origin of reference frame                                            */
    /************************************************************************/

    Eigen::Vector3f global_joint_origin;

    Eigen::Vector3f x_axis_unit_point;
    Eigen::Vector3f y_axis_unit_point;
    Eigen::Vector3f z_axis_unit_point;

    /************************************************************************/
    /* DH parametrs                                                         */
    /************************************************************************/

    float d;
    float theta;

    /************************************************************************/
    /* Joint type                                                           */
    /************************************************************************/

    JointT current_joint_type;

public:

    Joint(float d_in , float theta_in , JointT jt_in , std::string jname = "unnamed");

    void rotateJoint(const float& rot_angle);
    void setJoint(const float& rot_angle);
    void tranlsateJoint(float & displacement);
    JointT getJointType() { return current_joint_type; }
    float giveMeVariableParametr();

    float& getDisplasmentParametr_d(){ return d; }
    float& getRotationParametr_theta(){ return theta; }

    std::string& getJointName(){ return joint_name; }
    void setGlobalPosition(const Eigen::Vector3f & vec);
};


#endif /* Joint_hpp */
