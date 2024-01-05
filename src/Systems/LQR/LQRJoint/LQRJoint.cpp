////
//// Created by cc on 27/01/23.
////
//
//#include "LQRJoint.h"
//
//LQRJoint::LQRJoint(const LQRJointConfig &config) : WPISubsystem("LQRJoint"),
//plant(frc::LinearSystemId::IdentifyPositionSystem<units::radian>(config.kV / 1_rad_per_s,
//                                                                  config.kA / 1_rad_per_s_sq)),
//                                                   observer(plant,
//                                                            {0.0, 0.0},
//                                                            {0.0},
//                                                            20_ms),
//                                                   controller(plant,
//                                                              {0.0, 0.0},
//                                                              {0.0},
//                                                              20_ms),
//                                                              loop{
//
//}