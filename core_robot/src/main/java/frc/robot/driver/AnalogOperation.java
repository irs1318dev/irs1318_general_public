package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum AnalogOperation implements IOperation
{
    PositionStartingAngle,

    // DriveTrain operations:
    DriveTrainMoveForward,
    DriveTrainTurn,
    DriveTrainLeftPosition,
    DriveTrainRightPosition,
    DriveTrainLeftVelocity,
    DriveTrainRightVelocity,
    DriveTrainHeadingCorrection,
    DriveTrainStartingXPosition,
    DriveTrainStartingYPosition;
}
