//
// Created by cc on 1/11/23.
//

#ifndef BOTBUSTERS_REBIRTH_TOGGLEBOOL_H
#define BOTBUSTERS_REBIRTH_TOGGLEBOOL_H

//https://github.com/Team254/FRC-2023-Public/tree/main/src/main/java/com/team254/lib/util


//on true it toggles the internal value
class ToggleBool {
public:
    ToggleBool();

    bool update(bool val);

private:
    bool released = true;
    bool retVal = false;
};


#endif //BOTBUSTERS_REBIRTH_TOGGLEBOOL_H
