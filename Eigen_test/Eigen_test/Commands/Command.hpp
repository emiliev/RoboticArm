//
//  Command.hpp
//  Eigen_test
//
//  Created by Emil-Iliev on 5.02.20.
//  Copyright © 2020 Emil Iliev. All rights reserved.
//

#ifndef Command_hpp
#define Command_hpp

#include <stdio.h>
#include <string>

class Command {
public:
    virtual void execute() = 0;
    virtual std::string key() = 0;
};

#endif /* Command_hpp */
