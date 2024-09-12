package frc.robot;

import frc.lib.helpers.Helpers;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    public static final double MAX_ROBOT_HEIGHT = 45.5;//44.75; // inches, max overall height
    public static final double MAX_ROBOT_EXTENSION = 10.0;//9.75; // inches, max extension beyond frame perimeter
    public static final double ROBOT_FRAME_DIMENSION = 28.0; // frame perimeter / 4.0
    public static final double ROBOT_HALF_FRAME_PERIMETER = 17.0; // "half frame dimension" + 3.0"

    //================================================== DriveTrain ==============================================================
    // Note: Pulse Distance is the distance moved per tick

    public static final double TANK_DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION = 2048.0;
    public static final double TANK_DRIVETRAIN_LEFT_GEAR_RATIO = 8.45864662;
    public static final double TANK_DRIVETRAIN_LEFT_WHEEL_DIAMETER = 6.0; // (in inches)
    public static final double TANK_DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.TANK_DRIVETRAIN_LEFT_WHEEL_DIAMETER;
    public static final double TANK_DRIVETRAIN_LEFT_PULSE_DISTANCE = HardwareConstants.TANK_DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE / (HardwareConstants.TANK_DRIVETRAIN_LEFT_GEAR_RATIO * HardwareConstants.TANK_DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION);
    public static final double TANK_DRIVETRAIN_LEFT_TICKS_PER_INCH = (HardwareConstants.TANK_DRIVETRAIN_LEFT_GEAR_RATIO * HardwareConstants.TANK_DRIVETRAIN_LEFT_ENCODER_PULSES_PER_REVOLUTION) / HardwareConstants.TANK_DRIVETRAIN_LEFT_WHEEL_CIRCUMFERENCE;

    public static final double TANK_DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION = 2048.0;
    public static final double TANK_DRIVETRAIN_RIGHT_GEAR_RATIO = 8.45864662;
    public static final double TANK_DRIVETRAIN_RIGHT_WHEEL_DIAMETER = 6.0; // (in inches)
    public static final double TANK_DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.TANK_DRIVETRAIN_RIGHT_WHEEL_DIAMETER;
    public static final double TANK_DRIVETRAIN_RIGHT_PULSE_DISTANCE = HardwareConstants.TANK_DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE / (HardwareConstants.TANK_DRIVETRAIN_RIGHT_GEAR_RATIO * HardwareConstants.TANK_DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION);
    public static final double TANK_DRIVETRAIN_RIGHT_TICKS_PER_INCH = (HardwareConstants.TANK_DRIVETRAIN_RIGHT_GEAR_RATIO * HardwareConstants.TANK_DRIVETRAIN_RIGHT_ENCODER_PULSES_PER_REVOLUTION) / HardwareConstants.TANK_DRIVETRAIN_RIGHT_WHEEL_CIRCUMFERENCE;

    // measure from outside of wheel:
    public static final double TANK_DRIVETRAIN_WHEEL_SEPARATION_DISTANCE = 24.75; // (in inches)

    // DriveTrain motor/sensor orientations
    public static final boolean TANK_DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT = false;
    public static final boolean TANK_DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT = false;
    //public static final boolean TANK_DRIVETRAIN_LEFT_FOLLOWER2_INVERT_OUTPUT = false;
    //public static final boolean TANK_DRIVETRAIN_LEFT_INVERT_SENSOR = false;
    public static final boolean TANK_DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT = true;
    public static final boolean TANK_DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT = true;
    //public static final boolean TANK_DRIVETRAIN_RIGHT_FOLLOWER2_INVERT_OUTPUT = true;
    //public static final boolean TANK_DRIVETRAIN_RIGHT_INVERT_SENSOR = true;
}
