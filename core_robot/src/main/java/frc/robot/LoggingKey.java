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
    OffboardVisionX("rpi/x", LoggingType.NullableNumber, true, 1),
    OffboardVisionY("rpi/y", LoggingType.NullableNumber, true, 1),
    OffboardVisionWidth("rpi/width", LoggingType.NullableNumber, true, 1),
    OffboardVisionHeight("rpi/height", LoggingType.NullableNumber, true, 1),
    OffboardVisionAngle("rpi/angle", LoggingType.NullableNumber, true, 1),
    OffboardVisionDistance("rpi/distance", LoggingType.NullableNumber, true, 1),
    OffboardVisionHorizontalAngle("rpi/horizontalAngle", LoggingType.NullableNumber, true, 1),
    OffboardVisionEnableVision("rpi/enableVision", LoggingType.Boolean, false, 1),
    OffboardVisionEnableStream("rpi/enableStream", LoggingType.Boolean, false, 1),
    OffboardVisionEnableProcessing("rpi/enableProcessing", LoggingType.Number, false, 1),
    OffboardVisionMissedHeartbeats("rpi/missedHeartbeats", LoggingType.Number, true, 1),
    NavxConnected("navx/connected", LoggingType.Boolean, true),
    NavxAngle("navx/angle", LoggingType.Number, true),
    NavxPitch("navx/pitch", LoggingType.Number, true),
    NavxRoll("navx/roll", LoggingType.Number, true),
    NavxYaw("navx/yaw", LoggingType.Number, true),
    NavxX("navx/x", LoggingType.Number, true),
    NavxY("navx/y", LoggingType.Number, true),
    NavxZ("navx/z", LoggingType.Number, true),
    NavxStartingAngle("navx/startingAngle", LoggingType.Number, false),
    PigeonState("pigeon/state", LoggingType.String, false),
    PigeonYaw("pigeon/yaw", LoggingType.Number, true),
    PigeonPitch("pigeon/pitch", LoggingType.Number, true),
    PigeonRoll("pigeon/roll", LoggingType.Number, true),
    PigeonStartingYaw("pigeon/startingYaw", LoggingType.Number, false),

    DriveTrainDesiredAngle("dt/angle_goal", LoggingType.Number, false),
    DriveTrainAngle("dt/angle", LoggingType.Number, false),
    DriveTrainXPosition("dt/xpos", LoggingType.Number, false),
    DriveTrainYPosition("dt/ypos", LoggingType.Number, false),
    DriveTrainFieldOriented("dt/field", LoggingType.Boolean, false),
    DriveTrainMaintainPosition("dt/maintain_pos", LoggingType.Boolean, false),

    DriveTrainAbsoluteEncoderAngle1("dt/absenc_ang1", LoggingType.Number, true),
    DriveTrainDriveVelocity1("dt/drive_vel1", LoggingType.Number, true),
    DriveTrainDrivePosition1("dt/drive_pos1", LoggingType.Number, true),
    DriveTrainDriveError1("dt/drive_err1", LoggingType.Number, true),
    DriveTrainDriveVelocityGoal1("dt/drive_goal1", LoggingType.Number, false),
    DriveTrainSteerVelocity1("dt/steer_vel1", LoggingType.Number, true),
    DriveTrainSteerPosition1("dt/steer_pos1", LoggingType.Number, true),
    DriveTrainSteerAngle1("dt/steer_ang1", LoggingType.Number, false),
    DriveTrainSteerError1("dt/steer_err1", LoggingType.Number, false),
    DriveTrainSteerPositionGoal1("dt/steer_goal1", LoggingType.NullableNumber, false),

    DriveTrainAbsoluteEncoderAngle2("dt/absenc_ang2", LoggingType.Number, true),
    DriveTrainDriveVelocity2("dt/drive_vel2", LoggingType.Number, true),
    DriveTrainDrivePosition2("dt/drive_pos2", LoggingType.Number, true),
    DriveTrainDriveError2("dt/drive_err2", LoggingType.Number, true),
    DriveTrainDriveVelocityGoal2("dt/drive_goal2", LoggingType.Number, false),
    DriveTrainSteerVelocity2("dt/steer_vel2", LoggingType.Number, true),
    DriveTrainSteerPosition2("dt/steer_pos2", LoggingType.Number, true),
    DriveTrainSteerAngle2("dt/steer_ang2", LoggingType.Number, false),
    DriveTrainSteerError2("dt/steer_err2", LoggingType.Number, false),
    DriveTrainSteerPositionGoal2("dt/steer_goal2", LoggingType.NullableNumber, false),

    DriveTrainAbsoluteEncoderAngle3("dt/absenc_ang3", LoggingType.Number, true),
    DriveTrainDriveVelocity3("dt/drive_vel3", LoggingType.Number, true),
    DriveTrainDrivePosition3("dt/drive_pos3", LoggingType.Number, true),
    DriveTrainDriveError3("dt/drive_err3", LoggingType.Number, true),
    DriveTrainDriveVelocityGoal3("dt/drive_goal3", LoggingType.Number, false),
    DriveTrainSteerVelocity3("dt/steer_vel3", LoggingType.Number, true),
    DriveTrainSteerPosition3("dt/steer_pos3", LoggingType.Number, true),
    DriveTrainSteerAngle3("dt/steer_ang3", LoggingType.Number, false),
    DriveTrainSteerError3("dt/steer_err3", LoggingType.Number, false),
    DriveTrainSteerPositionGoal3("dt/steer_goal3", LoggingType.NullableNumber, false),

    DriveTrainAbsoluteEncoderAngle4("dt/absenc_ang4", LoggingType.Number, true),
    DriveTrainDriveVelocity4("dt/drive_vel4", LoggingType.Number, true),
    DriveTrainDrivePosition4("dt/drive_pos4", LoggingType.Number, true),
    DriveTrainDriveError4("dt/drive_err4", LoggingType.Number, true),
    DriveTrainDriveVelocityGoal4("dt/drive_goal4", LoggingType.Number, false),
    DriveTrainSteerVelocity4("dt/steer_vel4", LoggingType.Number, true),
    DriveTrainSteerPosition4("dt/steer_pos4", LoggingType.Number, true),
    DriveTrainSteerAngle4("dt/steer_ang4", LoggingType.Number, false),
    DriveTrainSteerError4("dt/steer_err4", LoggingType.Number, false),
    DriveTrainSteerPositionGoal4("dt/steer_goal4", LoggingType.NullableNumber, false),

    PowerCellIsIntaking("pc/intaking", LoggingType.Boolean, false),
    PowerCellIntakeExtended("pc/intake_extended", LoggingType.Boolean, false),
    PowerCellFlywheelVelocity("pc/flywheel_vel", LoggingType.Number, true),
    PowerCellFlywheelPosition("pc/flywheel_pos", LoggingType.Number, true),
    PowerCellFlywheelError("pc/flywheel_err", LoggingType.Number, true),
    PowerCellCarouselPower("pc/carousel_power", LoggingType.Number, false),
    PowerCellDesiredCarouselVelocity("pc/carousel_des_vel", LoggingType.Number, false),
    PowerCellFlywheelVelocitySetpoint("pc/flywheel_vel_sp", LoggingType.Number, false),

    PowerCellCarouselVelocity("pc/carousel_vel", LoggingType.Number, true),
    PowerCellCarouselPosition("pc/carousel_pos", LoggingType.Number, true),

    ControlPanelExtend("cp/extend", LoggingType.Boolean, false);

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
