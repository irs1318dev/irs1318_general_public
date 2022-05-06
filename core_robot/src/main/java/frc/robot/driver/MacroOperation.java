package frc.robot.driver;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // Path testing
    FollowPathTest1,
    FollowPathTest2,

    // DriveTrain operations:
    PIDBrake,

    // Vision operations
    VisionCenterHub,
    VisionCenterCargo,
}
