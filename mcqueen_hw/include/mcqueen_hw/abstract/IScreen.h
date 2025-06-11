#ifndef ISCREEN_H
#define ISCREEN_H

#include <string>

class IScreen {
public:
    virtual ~IScreen();

    virtual void mirrorScreen(const std::string& payload) = 0;
};

#endif // ISCREEN_H
