//
//  Joint.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#include "Joint.hpp"

void Joint::rotateJoint( const float& rot_angle ) {
    theta += rot_angle;
}

void Joint::tranlsateJoint( float& displacement ) {
    d+=displacement;
}

Joint::Joint( float d_in , float theta_in , JointT jt_in , std::string jname)
                                                            :d(d_in),
                                                            theta(theta_in),
                                                            current_joint_type(jt_in),
                                                            joint_name(jname){
}

void Joint::setGlobalPosition( const Eigen::Vector3f& vec ) {
    global_joint_origin = vec;
}

float Joint::giveMeVariableParametr() {
    switch(current_joint_type) {
    case REVOLUTE:
        return theta;
    case PRISMATIC:
        return d;
    }

    return 0.0f;
}
