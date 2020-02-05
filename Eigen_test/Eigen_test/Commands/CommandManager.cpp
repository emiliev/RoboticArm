//
//  CommandManager.cpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#include "CommandManager.hpp"

CommandManager::CommandManager() {
}

void CommandManager::addCommand(std::shared_ptr<Command> command) {
    std::string key = command->key();
    commands.insert(std::make_pair(key, command));
}

bool CommandManager::executeCommand(std::string commandKey) {
    const auto iter = commands.find(commandKey);
    bool result = iter != commands.end();
    if (result) {
        iter->second->execute();
    }
    return result;
}
