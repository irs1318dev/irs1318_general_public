package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDBrake,

    // Testing operations
    PowerCellFlywheelVelocity,
    VisionCenter,
    VisionCenterAndAdvance,
    FollowSomePath,
    FollowAnotherPath,
    FollowADifferentPath,
    ShootHopper,
    SpinFlywheel,
}
