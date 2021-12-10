package frc.robot;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r.state", true),
    RobotTime("r.time", true),
    RobotMatch("r.match"),
    DriverMode("driver.mode"),
    DriverActiveMacros("driver.activeMacros", true),
    DriverActiveShifts("driver.activeShifts"),
    AutonomousSelection("auto.selected"),
    AutonomousDSMessage("auto.dsMessage"),
    OffboardVisionX("rpi.x", true),
    OffboardVisionY("rpi.y", true),
    OffboardVisionWidth("rpi.width", true),
    OffboardVisionHeight("rpi.height", true),
    OffboardVisionAngle("rpi.angle", true),
    OffboardVisionDistance("rpi.distance", true),
    OffboardVisionHorizontalAngle("rpi.horizontalAngle", true),
    OffboardVisionEnableVision("rpi.enableVision", true),
    OffboardVisionEnableStream("rpi.enableStream", true),
    OffboardVisionEnableProcessing("rpi.enableProcessing", true), // why do some have True?
    OffboardVisionMissedHeartbeats("rpi.missedHeartbeats", true),
    NavxConnected("navx.connected", true),
    NavxAngle("navx.angle", true),
    NavxPitch("navx.pitch", true),
    NavxRoll("navx.roll", true),
    NavxYaw("navx.yaw", true),
    NavxX("navx.x"),
    NavxY("navx.y"),
    NavxZ("navx.z"),
    NavxStartingAngle("navx.startingAngle"),
    PigeonState("pigeon.state"),
    PigeonYaw("pigeon.yaw"),
    PigeonPitch("pigeon.pitch"),
    PigeonRoll("pigeon.roll"),
    PigeonStartingYaw("pigeon.startingYaw"),


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
