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

    // Vision operations

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,

    // Wrist operations:
    LowCubeDrop,
    MidCubeDrop,
    HighCubeDrop,
    WristStowed,
    SubstationPickup,
    GroundPickup,
}
