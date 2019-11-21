//
//  RobotType.h
//  Eigen_test
//
//  Created by Emil Iliev on 17.10.19.
//  Copyright © 2019 Emil Iliev. All rights reserved.
//

#ifndef RobotType_h
#define RobotType_h

#include "Link.hpp"
#include "Joint.hpp"

typedef std::vector<Joint> JointHandler;
typedef std::vector<Link> LinkHandler;

enum {
    //This parameter is about how many variable used for end effector
    //position and orientation : 3 for position, 3 for orientation
    NUMBEROFSETPARAMETERS = 6
};


#endif /* RobotType_h */
