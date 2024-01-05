//
// Created by cc on 24/07/23.
//

#include "LedsWithDIO.h"

LedsWithDIO::LedsWithDIO(const LedsWithDIOConfig &config) {
    this->config = config;

    for (int pinID : config.pinIDs) {
        auto item = std::make_shared<frc::DigitalOutput>(pinID);
        outputs.emplace_back(item);
    }


}

std::vector<bool> LedsWithDIO::intToBinary(int in) const{
    switch (in) {
        case 0:
            return {false, false, false};
        case 1:
            return {false, false, true};
        case 2:
            return {false, true, false};
        case 3:
            return {false, true, true};
        case 4:
            return {true, false, false};
        case 5:
            return {true, false, true};
        case 6:
            return {true, true, false};
        case 7:
            return {true, true, true};
        default:
            throw std::runtime_error("the binary value went over value");
            return {};
    }
}

std::string LedsWithDIO::decideEffect(bool init,
                                      bool enable,
                                      bool piece,
                                      bool inIdle,
                                      bool error,
                                      bool wristInTol,
                                      bool arcInTol,
                                      bool visionAligned) const {
    if (error) {
        return "red";
    } else if (visionAligned) {
        return "green";
    } else if (enable && piece) {
        return "orange";
    } else if (enable && !piece) {
        return "purple";
    } else if (init && inIdle) {
        return "green";
    } else if (!wristInTol && !arcInTol) {
        return "bothWhite";
    } else if (!arcInTol) {
        return "arcWhite";
    } else if (!wristInTol) {
        return "wristWhite";
    }else {
        return "green";
    }
}

void LedsWithDIO::setEffects(std::string set) const{
    int ints = config.colorsToInts.find(set)->second;
    auto bits = intToBinary(ints);
    int i = 0;
    for (auto &out: outputs){
        out->Set(bits[i]);
        i++;
    }
}
