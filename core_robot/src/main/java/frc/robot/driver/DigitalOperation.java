package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,
    PositionResetRobotLevel,

    // Driver interaction operations
    ForceLightDriverRumble,

    // Vision operations:
    VisionForceDisable,
    VisionEnableStream,
    VisionFindSpecificAprilTagRear,
    VisionFindSpecificAprilTagFront,
    VisionFindAnyAprilTagRear,
    VisionFindAnyAprilTagFront,
    VisionFindAbsolutePosition,

    // Compressor operations:
    CompressorForceDisable,

    // DriveTrain operations:
    DriveTrainSlowMode,
    DriveTrainEnablePID,
    DriveTrainDisablePID,
    DriveTrainSimpleMode,
    DriveTrainUseBrakeMode,
    DriveTrainUsePositionalMode,
    DriveTrainUsePathMode,
    DriveTrainSwapFrontOrientation,
    DriveTrainResetXYPosition,

    // Intake operations:
    IntakeExtend,
    IntakeRetract,
    IntakeRotatingIn,
    IntakeRotatingOut,

    // Stinger operations:
    StingerOut,
    StingerIn,

    // Shooter operations:
    ShooterSpin,
    ShooterLowerKicker,
    ShooterExtendHood,
}
