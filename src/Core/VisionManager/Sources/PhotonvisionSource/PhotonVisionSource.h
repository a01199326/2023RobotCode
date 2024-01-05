//
// Created by abiel on 2/25/22.
//

#ifndef BOTBUSTERS_REBIRTH_PHOTONVISIONSOURCE_H
#define BOTBUSTERS_REBIRTH_PHOTONVISIONSOURCE_H

#include "Core/VisionManager/Sources/VisionSource.h"
#include "Math/VisionUtilities.h"
#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonCamera.h>
#include "Systems/EctoSwerve/EctoSwerve.h"

class PhotonVisionSource : public VisionSource {
public:
    PhotonVisionSource(const CameraInfo &info, const std::shared_ptr<EctoSwerve> &swerve);

    void setLeds(LEDState state) override;

    [[nodiscard]] CameraInfo getCameraInfo() const override;

    [[nodiscard]] TrackedTarget getTrackedTarget() const override;

    [[nodiscard]] NeuralDetectorResults getNeuralResults() const override;

    [[nodiscard]] AprilTagResults getAprilTagResults() const override;

    void setPipeline(int pipelineId) const;


private:

    std::shared_ptr<EctoSwerve> swerve;

    std::shared_ptr<photonlib::PhotonCamera> camera;

    CameraInfo info;

    photonlib::PhotonPipelineResult prevResult;
};


#endif //BOTBUSTERS_REBIRTH_PHOTONVISIONSOURCE_H
