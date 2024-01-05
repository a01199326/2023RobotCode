//
// Created by cc on 1/11/23.
//

#ifndef BOTBUSTERS_REBIRTH_LATCHEDBOOL_H
#define BOTBUSTERS_REBIRTH_LATCHEDBOOL_H

//https://github.com/Team254/FRC-2023-Public/tree/main/src/main/java/com/team254/lib/util
class LatchedBool {
public:
    LatchedBool();

    bool update(bool val);

private:
    bool last = false;
};


#endif //BOTBUSTERS_REBIRTH_LATCHEDBOOL_H
