//
// Created by cc on 23/01/23.
//

#ifndef BOTBUSTERS_REBIRTH_PIDTELESCOPIC_H
#define BOTBUSTERS_REBIRTH_PIDTELESCOPIC_H

#include "Core/MotorHandler/MotorManager.h"
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include "Utilities/NetworkTablePID/NetworkTablePID.h"
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <frc/DigitalInput.h>
#include "Core/EctoModule/System.h"
#include <frc/DoubleSolenoid.h>
#include <units/power.h>
#include "Core/EctoModule/WPISubsystem.h"
#include "Systems/Generic/GearBox/GearBox.h"


struct PIDTelescopicConfig {
    PIDConfig pidConfig;

    std::vector<std::shared_ptr<EctoMotor>> motors;
    std::vector<bool> isInverted;

    units::ampere_t currentLimit = 20_A;
    double rampRate = 0.1;

    double gearReduction = 1; //Motor Rotations / Rotations Out
    units::meter_t pulleyDiameter = 0_m;

    frc::ElevatorFeedforward freeFF {0.0_V, 0.0_V, 0.0_V / 1_mps, 0.0_V / 1_mps_sq};
    frc::ElevatorFeedforward loadedFF {0.0_V, 0.0_V, 0.0_V / 1_mps, 0.0_V / 1_mps_sq};

    bool enableLimitSwitch = true;
    bool enableReverseSoftLimit = true;
    bool enableForwardSoftLimit = true;

    units::meter_t reverseSoftLimit = 0_m;
    units::meter_t forwardSoftLimit = 0_m;

    units::meters_per_second_t maxVelocity = 1_mps;
    units::meters_per_second_squared_t maxAcceleration = 0.5_mps_sq;

    units::meter_t pidDistTol = 1_m;
    units::meters_per_second_t pidVelTol = 1_mps;

    int stages = 1;

    units::meters_per_second_t maxGoingUpTeleopVel = 1_mps;
    units::meters_per_second_squared_t maxGoingUpTeleopAccel = 0.5_mps_sq;

    units::meters_per_second_t maxGoingDownTeleopVel = 1_mps;
    units::meters_per_second_squared_t maxGoingDownTeleopAccel = 0.5_mps_sq;

    units::meters_per_second_t maxGoingUpAutoVel = 1_mps;
    units::meters_per_second_squared_t maxGoingUpAutoAccel = 0.5_mps_sq;

    units::meters_per_second_t maxGoingDownAutoVel = 1_mps;
    units::meters_per_second_squared_t maxGoingDownAutoAccel = 0.5_mps_sq;

};

class PIDTelescopic : public WPISubsystem {
public:
    enum class FeedForward{
        Loaded,
        Free
    };

    explicit PIDTelescopic(const PIDTelescopicConfig &config);

    void robotInit() override;

    void robotUpdate() override;

    void setPIDConfig(const PIDConfig &pidConfig);

    units::meter_t getHeight() const;

    units::meters_per_second_t getVel() const;

    void set(units::meter_t height);

    void set(double height){
        set(units::meter_t(height));
    }

    void enableTelescopic(bool state){ this->enable = state;};

    void setVoltage(double setVoltage);

    bool getLimitSwitchState();

    void setUsePID(bool usePID);

    void resetToZero();

    bool atGoal();

    bool inTol(units::meter_t tol);

    units::meters_per_second_t getMaxVel() const;

    units::meters_per_second_squared_t  getMaxAccel() const;

    void setConstraints(const units::meters_per_second_t maxSpeed, const units::meters_per_second_squared_t maxAccel);

    void setFF(frc::ElevatorFeedforward ff);

    void useSoftLimits(bool useSoftLimits);

    void resetController(units::meter_t height);

    units::meter_t getGoal() {return goal;}

    PIDTelescopicConfig getConfig() const;

private:

    void updateTelemetry(double out);

    [[nodiscard]] units::meter_t radToHeight(units::radian_t rads) const;

    [[nodiscard]] units::radian_t heightToRad(units::meter_t height) const;

    PIDTelescopicConfig config;
    std::vector<std::shared_ptr<EctoMotor>> motors;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;
    nt::NetworkTableEntry masterMotorCurrent, slaveMotorCurrent;

    std::unique_ptr<frc::ProfiledPIDController<units::meters>> pidController;
    frc::ElevatorFeedforward* ff{};

    bool usePID{true};

    bool useElevator{true};

    units::meter_t goal;

    double manualVoltage{};

    bool enable{true};

    bool hasHomed{false};
};


#endif //BOTBUSTERS_REBIRTH_PIDTELESCOPIC_H
