package frc.robot;

import frc.robot.common.robotprovider.TalonFXInvertType;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    //================================================== DriveTrain ==============================================================

    public static final TalonFXInvertType DRIVETRAIN_STEER_MOTOR1_INVERT = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType DRIVETRAIN_STEER_MOTOR2_INVERT = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType DRIVETRAIN_STEER_MOTOR3_INVERT = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType DRIVETRAIN_STEER_MOTOR4_INVERT = TalonFXInvertType.Clockwise;

    //================================================== DriveTrain ==============================================================
    // Note: Pulse Distance is the distance moved per tick

    public static final double DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION = 2048.0;
    public static final double DRIVETRAIN_LEFT_GEAR_RATIO = 8.45864662;
    public static final double DRIVETRAIN_LEFT_WHEEL_DIAMETER = 6.0; // (in inches)
    public static final double DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.DRIVETRAIN_LEFT_WHEEL_DIAMETER;
    public static final double DRIVETRAIN_LEFT_PULSE_DISTANCE = HardwareConstants.DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE / (HardwareConstants.DRIVETRAIN_LEFT_GEAR_RATIO * HardwareConstants.DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION);
    public static final double DRIVETRAIN_LEFT_TICKS_PER_INCH = (HardwareConstants.DRIVETRAIN_LEFT_GEAR_RATIO * HardwareConstants.DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION) / HardwareConstants.DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE;

    public static final double DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION = 2048.0;
    public static final double DRIVETRAIN_RIGHT_GEAR_RATIO = 8.45864662;
    public static final double DRIVETRAIN_RIGHT_WHEEL_DIAMETER = 6.0; // (in inches)
    public static final double DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_DIAMETER;
    public static final double DRIVETRAIN_RIGHT_PULSE_DISTANCE = HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE / (HardwareConstants.DRIVETRAIN_RIGHT_GEAR_RATIO * HardwareConstants.DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION);
    public static final double DRIVETRAIN_RIGHT_TICKS_PER_INCH = (HardwareConstants.DRIVETRAIN_RIGHT_GEAR_RATIO * HardwareConstants.DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION) / HardwareConstants.DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE;

    // measure from outside of wheel:
    public static final double DRIVETRAIN_WHEEL_SEPARATION_DISTANCE = 24.75; // (in inches)

    // DriveTrain motor/sensor orientations
    public static final boolean DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT = false;
    //public static final boolean DRIVETRAIN_LEFT_FOLLOWER2_INVERT_OUTPUT = false;
    public static final boolean DRIVETRAIN_LEFT_INVERT_SENSOR = false;
    public static final boolean DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT = true;
    public static final boolean DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT = true;
    //public static final boolean DRIVETRAIN_RIGHT_FOLLOWER2_INVERT_OUTPUT = true;
    public static final boolean DRIVETRAIN_RIGHT_INVERT_SENSOR = true;
}