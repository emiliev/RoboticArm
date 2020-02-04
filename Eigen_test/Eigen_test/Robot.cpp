//
//  Robot.cpp
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#include "Robot.hpp"
#include "MatrixFactory.hpp"
#include <iostream>

bool Robot::loadConfig( const dh_table& tbl ) {
    unsigned int sz = tbl.size();
    if(!sz) return false;

    robot_dh = tbl;

    for (unsigned int i = 0 ; i < sz ; ++i) {
        //Saving joint data to robot
        jhandle.push_back(Joint(robot_dh[i].d,robot_dh[i].theta,robot_dh[i].z_joint_type,robot_dh[i].joint_name));
        //Saving link data  to robot
        linkhadle.push_back(Link(robot_dh[i].a,robot_dh[i].alpha));
        //Calculating h_matrix and saving to robot
        auto matrix = matrix_algo->calculateHTranslationMatrix(
                                                               linkhadle[i].getZAxisRotationParametr_aplha(),
                                                               linkhadle[i].getCommonNormalParametr_a(),
                                                               jhandle[i].getDisplasmentParametr_d(),
                                                               jhandle[i].getRotationParametr_theta()
                                                               );
        hmtx.push_back(matrix);
    }

    caclulateFullTransormationMatrix();
    return true;
}

Robot::Robot( Eigen::Vector3f& vec ) : zero_origin(vec) , matrix_algo(MatrixFactory::getInstance()) , number_of_var_parameters(0) {
    
}

void Robot::setOrigin( Eigen::Vector3f & newOrigin ) {
    zero_origin = newOrigin;
}

bool Robot::rotateJoint( unsigned int ind , float angle ) {
    if (ind > jhandle.size()) {
        //Bad index
        return false;
    }
    
    Joint& current_joint = jhandle[ind];

    if (current_joint.getJointType() != REVOLUTE) {
        //We can rotate only revolute joint
        return false;
    }
    //TODO
    //add signal for this peace of code
    current_joint.rotateJoint(angle);
    //Recalculate HM for specified joint
    calculateJoint(ind);

    return true;
}

bool Robot::translateJoint( unsigned int ind , float displasment ) {
    if (ind > jhandle.size())
    {
        //Bad index
        return false;
    }
    Joint & current_joint = jhandle[ind];
    //We can translate only prismatic joint
    if (current_joint.getJointType() != PRISMATIC)
    {
        //We can translate only prismatic joint
        return false;
    }
    //TODO
    //add signal for this peace of code
    current_joint.tranlsateJoint(displasment);

    //Recalculate HM for specified joint
    calculateJoint(ind);

    return true;
}

bool Robot::printHomogenTransformationMatrix() {
    if(jhandle.size() != hmtx.size())
    {
        //TODO
        //May be i should add some extra info here
        std::cout<<"Number of Joints doesn't match number of ham. matrix. Suppose you have got extra for end-effector"<<std::endl;
        return false;
    }

    for (unsigned int i = 0 ; i < jhandle.size() ; ++i)
    {
        std::cout<<"Joint name : "<<jhandle[i].getJointName()<<std::endl<<"Joint ID : "<<i+1<<std::endl;
        std::cout<<"Matrix : "<<std::endl<<hmtx[i]<<std::endl<<"------------------------"<<std::endl;
    }

    return true;
}

bool Robot::caclulateFullTransormationMatrix() {
    if (hmtx.empty())
    {
        return false;
    }
    //For beginning we have identity matrix
    from_i_1_to_i = Eigen::Matrix4f::Identity();
    //TODO
    //i should add equation representation here
    //for more clearance
    for (unsigned int i = 0 ; i < hmtx.size() ; i++)
    {
        Eigen::Matrix4f _temp = from_i_1_to_i;
//        std::cout << hmtx[i] << std::endl;
        from_i_1_to_i = _temp * hmtx[i];
    }

    return true;
}

bool Robot::printFullTransformationMatrix() {
    std::cout<<"Full HM matrix :"<<std::endl<<from_i_1_to_i<<std::endl;
    return true;
}

bool Robot::calculateNumberOfVariableParametrs() {
    if(jhandle.empty())
        return false;

    for (unsigned int i = 0 ; i < jhandle.size() ; ++i)
    {
        if (jhandle[i].getJointType() != CONSTANTJOINT)
        {
            number_of_var_parameters++;
        }
    }

    return true;
}

bool Robot::setJointVariable(unsigned int ind,float new_var) {
    bool retval = true;

    switch(jhandle[ind].getJointType())
    {
    case REVOLUTE:
        jhandle[ind].getRotationParametr_theta() = new_var;
        break;
    case PRISMATIC:
        jhandle[ind].getDisplasmentParametr_d() = new_var;
        break;
    default:
        retval = false;
    }

    if (retval)
    {
        calculateJoint(ind);
    }

    return retval;
}

bool Robot::calculateJoint( unsigned int ind ) {
    //TODO
    //add checking
    //Recalculate HM for specified joint
    hmtx[ind] = matrix_algo->calculateHTranslationMatrix(
                                                            linkhadle[ind].getZAxisRotationParametr_aplha(),
                                                            linkhadle[ind].getCommonNormalParametr_a(),
                                                            jhandle[ind].getDisplasmentParametr_d(),
                                                            jhandle[ind].getRotationParametr_theta()
                                                         );

    caclulateFullTransormationMatrix();
    return true;
}

void Robot::printConfiguration() {
    unsigned int sz = jhandle.size();

    for (unsigned int i = 0 ; i < sz ; ++i) {
        std::cout<<"Joint name : "<<jhandle[i].getJointName()<<std::endl;
        float jvar = jhandle[i].giveMeVariableParametr();
        
        switch(jhandle[i].getJointType()) {
        case PRISMATIC:
            std::cout<<"Prismatic, VAR = ";
            break;
        case REVOLUTE:
            std::cout<<"Revolute, VAR = ";
            break;
        default:
            break;
        }
        std::cout<<jvar<<std::endl;
    }

    float x_pos = from_i_1_to_i(0,3);
    float y_pos = from_i_1_to_i(1,3);
    float z_pos = from_i_1_to_i(2,3);

    std::cout<<"Position :"<<std::endl;
    std::cout<<"X : "<<x_pos<<std::endl;
    std::cout<<"Y : "<<y_pos<<std::endl;
    std::cout<<"Z : "<<z_pos<<std::endl;
}
