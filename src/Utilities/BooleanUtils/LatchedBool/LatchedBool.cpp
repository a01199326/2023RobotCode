//
// Created by cc on 1/11/23.
//

#include "LatchedBool.h"

LatchedBool::LatchedBool() {
    ;
}

bool LatchedBool::update(bool val) {
    bool ret = false;
    if (val && !last) {
        ret = true;
    }
    last = val;
    return ret;
}