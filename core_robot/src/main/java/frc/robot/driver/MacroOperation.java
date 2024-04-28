package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,
    FaceForward,
    FaceBackward,
    FaceLeft,
    FaceRight,
    FaceSomething,

    // Vision operations

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,
}
