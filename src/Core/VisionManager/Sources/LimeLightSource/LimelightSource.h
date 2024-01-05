//
// Created by abiel on 2/25/22.
//

#ifndef BOTBUSTERS_REBIRTH_LIMELIGHTSOURCE_H
#define BOTBUSTERS_REBIRTH_LIMELIGHTSOURCE_H

#include "Core/VisionManager/Sources/VisionSource.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "LimeLight.h"
#include "Systems/EctoSwerve/EctoSwerve.h"


class LimelightSource : public VisionSource {
public:
    LimelightSource(const CameraInfo &info, const std::shared_ptr<EctoSwerve> &swerve);

    void setLeds(LEDState state) override;

    [[nodiscard]] CameraInfo getCameraInfo() const override;

    [[nodiscard]] TrackedTarget getTrackedTarget() const override;

    [[nodiscard]] NeuralDetectorResults getNeuralResults() const override;

    [[nodiscard]] AprilTagResults getAprilTagResults() const override;

    void setPipeline(int pipelineId) const override;

private:

    std::shared_ptr<EctoSwerve> swerve;

    std::shared_ptr<LimeLight> camera;

    CameraInfo info;

    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("limelight");


};


#endif //BOTBUSTERS_REBIRTH_LIMELIGHTSOURCE_H
