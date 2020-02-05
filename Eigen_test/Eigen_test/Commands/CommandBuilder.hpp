//
//  CommandBuilder.hpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef CommandBuilder_hpp
#define CommandBuilder_hpp

#include <stdio.h>
#include "CommandManager.hpp"
class Robot;
class AbstractSolver;

class CommandManagerBuilder {
public:
    static std::shared_ptr<CommandManager> createManager(std::shared_ptr<Robot> currentRobot, std::shared_ptr<AbstractSolver> currentSolver);
};

#endif /* CommandBuilder_hpp */
