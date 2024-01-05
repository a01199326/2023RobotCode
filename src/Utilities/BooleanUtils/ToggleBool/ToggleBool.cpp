//
// Created by cc on 1/11/23.
//

#include "ToggleBool.h"

ToggleBool::ToggleBool() {;}

bool ToggleBool::update(bool val) {
    if (val && released) {
        released = false;
        retVal = !retVal;
    }
    if (!val) {
        released = true;
    }
    return retVal;
}