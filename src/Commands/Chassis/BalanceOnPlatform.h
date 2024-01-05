//
// Created by cc on 11/02/23.
//

#ifndef BOTBUSTERS_REBIRTH_BALANCEONPLATFORM_H
#define BOTBUSTERS_REBIRTH_BALANCEONPLATFORM_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <frc/filter/LinearFilter.h>

class BalanceOnPlatform : public frc2::CommandHelper<frc2::CommandBase, BalanceOnPlatform>{
public:
    explicit BalanceOnPlatform(const std::shared_ptr<EctoSwerve> &swerve);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:

    double bangBangHysterisis(double in);

    std::shared_ptr<EctoSwerve> swerve;
    frc2::PIDController pitchController, rollController;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table =
            ntInstance.GetTable("BalanceOnPlatform");


    [[nodiscard]] double getRotPitch() const;

    double pitchRate{}, prevPitch{};

    double uX{0}, uY{0};

    frc::LinearFilter<double> pitchFilter = frc::LinearFilter<double>::MovingAverage(2);
    frc::LinearFilter<double> ratePitchFilter = frc::LinearFilter<double>::MovingAverage(15);
};


#endif //BOTBUSTERS_REBIRTH_BALANCEONPLATFORM_H
