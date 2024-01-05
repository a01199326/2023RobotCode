//
// Created by cc on 24/07/23.
//

#ifndef BOTBUSTERS_REBIRTH_LEDSWITHDIO_H
#define BOTBUSTERS_REBIRTH_LEDSWITHDIO_H

#include <bits/stdc++.h>
#include <frc/DigitalOutput.h>


struct LedsWithDIOConfig {
    std::vector<int> pinIDs;
    std::map<std::string, int> colorsToInts;
};


class LedsWithDIO {
public:
    LedsWithDIO(const LedsWithDIOConfig &config);

    void setEffects(std::string set) const;

    std::string decideEffect(bool init, bool enable, bool piece, bool inIdle, bool error, bool wristInTol, bool arcInTol, bool visionAligned) const;

private:
    std::vector<bool> intToBinary(int in) const;

    LedsWithDIOConfig config;

    std::vector<std::shared_ptr<frc::DigitalOutput>> outputs;

};


#endif //BOTBUSTERS_REBIRTH_LEDSWITHDIO_H
