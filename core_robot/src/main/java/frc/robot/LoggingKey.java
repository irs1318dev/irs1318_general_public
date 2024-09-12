package frc.robot;

import frc.lib.robotprovider.LoggingType;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r/state", LoggingType.String, false, 1, true),
    RobotTime("r/time", LoggingType.Number, false, 1, true),
    RobotMatch("r/match", LoggingType.String, false, 50),
    RobotCrash("r/crash", LoggingType.String, false, true),
    DriverMode("driver/mode", LoggingType.String, false, 1, true),
    DriverActiveMacros("driver/activeMacros", LoggingType.String, false, 1, true),
    DriverActiveShifts("driver/activeShifts", LoggingType.String, false),
    AutonomousSelection("auto/selected", LoggingType.String, false),
    AutonomousDSMessage("auto/dsMessage", LoggingType.String, false),
    OffboardVisionAprilTagXOffset("vision/atXOffset", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagYOffset("vision/atYOffset", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagZOffset("vision/atZOffset", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagYaw("vision/atYaw", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagPitch("vision/atPitch", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagRoll("vision/atRoll", LoggingType.NullableNumber, true, 1),
    OffboardVisionAprilTagId("vision/atId", LoggingType.NullableInteger, true, 1),
    OffboardVisionProcessingMode("vision/processingMode", LoggingType.Integer, false, 1),
    OffboardVisionEnableStream("vision/enableStream", LoggingType.Boolean, false, 1),
    OffboardVisionDesiredTarget("vision/desiredTarget", LoggingType.String, false, 1),
    OffboardVisionMissedHeartbeats("vision/missedHeartbeats", LoggingType.Number, true, 1),
    OffboardVisionExcessiveMissedHeartbeats("vision/missedTooManyHeartbeats", LoggingType.Boolean, false, 1),
    PowerCurrent("power/curr", LoggingType.Number, true),
    PowerCurrentFloatingAverage("power/currFltAvg", LoggingType.Number, false),
    PowerBatteryVoltage("power/battV", LoggingType.Number, true),
    // PowerBatteryVoltageFiltered("power/battVFilt", LoggingType.Number, false),
    NavxStartingAngle("navx/startingAngle", LoggingType.Number, false),
    PigeonState("pigeon/state", LoggingType.String, true),
    PigeonYaw("pigeon/yaw", LoggingType.Number, true),
    PigeonPitch("pigeon/pitch", LoggingType.Number, true),
    PigeonPitchOffset("pigeon/pitchOffset", LoggingType.Number, false),
    PigeonRollOffset("pigeon/rollOffset", LoggingType.Number, false),
    PigeonYawOffset("pigeon/yawOffset", LoggingType.Number, false),
    PigeonRoll("pigeon/roll", LoggingType.Number, true),
    PigeonStartingYaw("pigeon/startingYaw", LoggingType.Number, false),
    PigeonYawRate("pigeon/yawRate", LoggingType.Number, true),
    PigeonPitchRate("pigeon/pitchRate", LoggingType.Number, true),
    PigeonRollRate("pigeon/rollRate", LoggingType.Number, true),
    NavxConnected("navx/isConnected", LoggingType.Boolean, true),
    NavxAngle("navx/angle", LoggingType.Number, true),
    NavxPitch("navx/pitch", LoggingType.Number, true),
    NavxRoll("navx/roll", LoggingType.Number, true),
    NavxYaw("navx/yaw", LoggingType.Number, true),
    NavxX("navx/x", LoggingType.Number, true),
    NavxY("navx/y", LoggingType.Number, true),
    NavxZ("navx/z", LoggingType.Number, true),

    DriveTrainLeftVelocity("dt/leftVelocity", LoggingType.Number, true),
    DriveTrainLeftTicks("dt/leftTicks", LoggingType.Number, true),
    DriveTrainLeftError("dt/leftError", LoggingType.Number, true),
    DriveTrainRightVelocity("dt/rightVelocity", LoggingType.Number, true),
    DriveTrainRightTicks("dt/rightTicks", LoggingType.Number, true),
    DriveTrainRightError("dt/rightError", LoggingType.Number, true),
    DriveTrainLeftMotorOut("dt/leftMotorOut", LoggingType.Number, false),
    DriveTrainRightMotorOut("dt/rightMotorOut", LoggingType.Number, false),
    DriveTrainLeftVelocityGoal("dt/leftVelocityGoal", LoggingType.Number, false),
    DriveTrainRightVelocityGoal("dt/rightVelocityGoal", LoggingType.Number, false),
    DriveTrainLeftPositionGoal("dt/leftPositionGoal", LoggingType.Number, false),
    DriveTrainRightPositionGoal("dt/rightPositionGoal", LoggingType.Number, false),

    DriveTrainXPosition("dt/x", LoggingType.Number, false),
    DriveTrainYPosition("dt/y", LoggingType.Number, false),

    CompressorPreassure("com/pres", LoggingType.Number, true),

    IntakeThroughBeam("in/throughBeam", LoggingType.Boolean, true),

    ShooterRate("s/rate", LoggingType.Number, true),
    ShooterTicks("s/ticks", LoggingType.Number, true),
    ShooterGoal("s/velGoal", LoggingType.Number, false),
    ShooterPower("s/power", LoggingType.Number, false);

    public final String value;
    public final LoggingType type;
    public final boolean isInput;
    public final int loggingFrequency;
    public final boolean shouldLogToCsv;
    private LoggingKey(String value, LoggingType type)
    {
        this(value, type, false, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput)
    {
        this(value, type, isInput, TuningConstants.DEFAULT_LOGGING_FREQUENCY, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, int loggingFrequency)
    {
        this(value, type, isInput, loggingFrequency, false);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, boolean shouldLogToCsv)
    {
        this(value, type, isInput, TuningConstants.DEFAULT_LOGGING_FREQUENCY, shouldLogToCsv);
    }

    private LoggingKey(String value, LoggingType type, boolean isInput, int loggingFrequency, boolean shouldLogToCsv)
    {
        if (loggingFrequency <= 0)
        {
            loggingFrequency = TuningConstants.DEFAULT_LOGGING_FREQUENCY;
        }

        this.value = value;
        this.type = type;
        this.isInput = isInput;
        this.loggingFrequency = loggingFrequency;
        this.shouldLogToCsv = shouldLogToCsv;
    }
}
