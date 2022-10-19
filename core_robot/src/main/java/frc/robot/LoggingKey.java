package frc.robot;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r.state", true),
    RobotTime("r.time", true),
    RobotMatch("r.match"),
    RobotCrash("r.crash", true),
    DriverMode("driver.mode"),
    DriverActiveMacros("driver.activeMacros", true),
    DriverActiveShifts("driver.activeShifts"),
    AutonomousSelection("auto.selected"),
    AutonomousDSMessage("auto.dsMessage"),
    OffboardVisionTargetDistance("rpi.v_distance", true),
    OffboardVisionTargetHorizontalAngle("rpi.v_horizontalAngle", true),
    OffboardVisionGamePieceDistance("rpi.g_distance", true),
    OffboardVisionGamePieceHorizontalAngle("rpi.g_horizontalAngle", true),
    OffboardVisionEnableVision("rpi.enableVision", true),
    OffboardVisionEnableStream("rpi.enableStream", true),
    OffboardVisionEnableProcessing("rpi.processingSetting", true),
    OffboardVisionMissedHeartbeats("rpi.missedHeartbeats", true),
    PowerCurrent("power.curr"),
    PowerCurrentFloatingAverage("power.currFltAvg"),
    PowerBatteryVoltage("power.battV"),
    NavxStartingAngle("navx.startingAngle"),
    PigeonState("pigeon.state"),
    PigeonYaw("pigeon.yaw", true),
    PigeonPitch("pigeon.pitch"),
    PigeonRoll("pigeon.roll"),
    PigeonStartingYaw("pigeon.startingYaw"),
    NavxConnected("navx.isConnected"),
    NavxAngle("navx.angle"),
    NavxPitch("navx.pitch"),
    NavxRoll("navx.roll"),
    NavxYaw("navx.yaw"),
    NavxX("navx.x"),
    NavxY("navx.y"),
    NavxZ("navx.z"),

    DriveTrainLeftVelocity("dt.leftVelocity", true),
    DriveTrainLeftTicks("dt.leftTicks", true),
    DriveTrainLeftError("dt.leftError", true),
    DriveTrainRightVelocity("dt.rightVelocity", true),
    DriveTrainRightTicks("dt.rightTicks", true),
    DriveTrainRightError("dt.rightError", true),
    DriveTrainLeftVelocityGoal("dt.leftVelocityGoal", true),
    DriveTrainRightVelocityGoal("dt.rightVelocityGoal", true),
    DriveTrainLeftPositionGoal("dt.leftPositionGoal", true),
    DriveTrainRightPositionGoal("dt.rightPositionGoal", true),

    DriveTrainXPosition("dt.x", true),
    DriveTrainYPosition("dt.y", true),

    CompressorPreassure("com.pres"),

    OneMotorSRXVelocity("omsrx.velocity"),
    OneMotorSRXError("omsrx.error"),
    OneMotorSRXPosition("omsrx.position"),
    OneMotorSRXForwardLimit("omsrx.forwardlimit"),
    OneMotorSRXReverseLimit("omsrx.reverselimit"),
    OneMotorSRXSetpoint("omsrx.setpoint"),
    OneMotorSRXErrorPercent("omsrx.error%"),
    OneMotorSparkVelocity("omspark.velocity"),
    OneMotorSparkPosition("omspark.position"),
    OneMotorSparkForwardLimit("omspark.forwardlimit"),
    OneMotorSparkReverseLimit("omspark.reverselimit"),
    OneMotorSparkSetpoint("omspark.setpoint");

    public final String value;
    public final boolean shouldLog;
    private LoggingKey(String value)
    {
        this(value, false);
    }

    private LoggingKey(String value, boolean shouldLog)
    {
        this.value = value;
        this.shouldLog = shouldLog;
    }
}
