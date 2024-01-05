//
// Created by cc on 11/02/23.
//

#include "PWMColorSensor.h"


PWMColorSensor::PWMColorSensor(const PWMColorSensorConfig &config) : WPISubsystem("colorSensor"){
    this->config = config;

    auto DIO = frc::DigitalInput(config.DIOPin);
    interrupt = std::make_shared<frc::SynchronousInterrupt>(DIO);
    interrupt->SetInterruptEdges(true, true);

    input.registerAxis(overrideCube, "leftTrigger", 0);
    input.registerAxis(overrideCone, "rightTrigger", 0);
}

void PWMColorSensor::robotInit() { ; }

void PWMColorSensor::robotUpdate() {
    if(overrideCube.get() > 0.4) {
        currentPiece = ScorePiece::CUBE;
        return;
    }

    if(overrideCone.get() > 0.4) {
        currentPiece = ScorePiece::CONE;
        return;
    }

//    auto piece = getDetectedPiece();
//    if (piece == ScorePiece::EMPTY){
//        return;
//    }
//
//    if (piece == ScorePiece::OVERLOAD && prevScorePiece != ScorePiece::OVERLOAD){
//        currentPiece = prevScorePiece;
//    }

//    prevScorePiece = piece;
}

bool PWMColorSensor::hasPiece(){
    auto detectedPiece = getDetectedPiece();
    return detectedPiece == ScorePiece::CUBE || detectedPiece == ScorePiece::CONE || detectedPiece == ScorePiece::OVERLOAD;
}

double PWMColorSensor::getRatio(){
    double rising = interrupt->GetRisingTimestamp().value();
    double falling = interrupt->GetFallingTimestamp().value();
    if (rising < falling){
//        prevRatio = adjustRatio((falling - rising) / config.pulsePeriod);
    } else {
        prevRatio = adjustRatio(1.0 + (falling - rising) / config.pulsePeriod);
    }
    return prevRatio;
}

double PWMColorSensor::adjustRatio(double value) const {
    double ret = (value - config.bound) / (1 -2 * config.bound);
    if (ret < 0.0 || ret > 1.0){
        return 0.3;
    }
    return ret;
}

bool PWMColorSensor::inRange(double in, double to, double tol) {
    return std::abs(to - in) < tol;
}

ScorePiece PWMColorSensor::getCurrentPiece(){
    return currentPiece;
}

ScorePiece PWMColorSensor::getDetectedPiece() {
    double ratio = getRatio();
    if (ratio == 0.3){
        return ScorePiece::EMPTY;
    }else{
        return ScorePiece::OVERLOAD;
    }

}