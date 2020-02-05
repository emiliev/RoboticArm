//
//  CommandManager.hpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef CommandManager_hpp
#define CommandManager_hpp

#include <stdio.h>
#include "Command.hpp"
#include <map>

class CommandManager {
    
public:
    CommandManager();
    void addCommand(std::shared_ptr<Command> command);
    bool executeCommand(std::string commandKey);
private:
    std::map<std::string, std::shared_ptr<Command>> commands;
    
};

#endif /* CommandManager_hpp */

