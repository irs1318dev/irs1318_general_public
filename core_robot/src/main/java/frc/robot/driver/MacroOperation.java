package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDBrake,
    FaceForward,
    FaceBackward,
    FaceLeft,
    FaceRight,
    FaceSomething,

    // Intake operations
    IntakeIn,
    IntakeOut,

    // Shooter operations
    SpinClose,
    SpinMiddle,
    SpinFar,
    Shoot,

    // Vision operations

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,
}
