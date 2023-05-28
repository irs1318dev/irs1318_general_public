package frc.robot;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    public static final double MAX_ROBOT_HEIGHT = 78.0; // inches, max overall height
    public static final double MAX_ROBOT_EXTENSION = 48.0; // inches, max extension beyond frame perimeter

    //================================================== DriveTrain ==============================================================

    public static final boolean DRIVETRAIN_STEER_MOTOR1_INVERT = true; // clockwise positive
    public static final boolean DRIVETRAIN_STEER_MOTOR2_INVERT = true; // clockwise positive
    public static final boolean DRIVETRAIN_STEER_MOTOR3_INVERT = true; // clockwise positive
    public static final boolean DRIVETRAIN_STEER_MOTOR4_INVERT = true; // clockwise positive

    public static final boolean DRIVETRAIN_DRIVE_MOTOR1_INVERT = false; // counter-clockwise positive
    public static final boolean DRIVETRAIN_DRIVE_MOTOR2_INVERT = false; // counter-clockwise positive
    public static final boolean DRIVETRAIN_DRIVE_MOTOR3_INVERT = false; // counter-clockwise positive
    public static final boolean DRIVETRAIN_DRIVE_MOTOR4_INVERT = false; // counter-clockwise positive

    public static final double DRIVETRAIN_STEER_GEAR_RATIO = 150.0 / 7.0; // According to SDS Mk4i code: (50.0 / 14.0) * (60.0 / 10.0) == ~21.43 : 1
    public static final double DRIVETRAIN_STEER_DEGREES = 360.0;
    public static final double DRIVETRAIN_STEER_TICK_DISTANCE = HardwareConstants.DRIVETRAIN_STEER_DEGREES / HardwareConstants.DRIVETRAIN_STEER_GEAR_RATIO; // in degrees
    public static final double DRIVETRAIN_STEER_TICKS_PER_DEGREE = HardwareConstants.DRIVETRAIN_STEER_GEAR_RATIO / HardwareConstants.DRIVETRAIN_STEER_DEGREES; // in rotations

    public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 36000.0 / 5880; // According to SDS Mk4i Very Fast code: (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0) == ~6.12 : 1
    public static final double DRIVETRAIN_DRIVE_WHEEL_DIAMETER = 3.725; // SDS Mk4i code claims their 4-inch wheels are actually 3.95 inches now (in inches) We think its 3.95 - main 3.97
    public static final double DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_DIAMETER;
    public static final double DRIVETRAIN_DRIVE_TICK_DISTANCE = HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE / HardwareConstants.DRIVETRAIN_DRIVE_GEAR_RATIO;
    public static final double DRIVETRAIN_DRIVE_TICKS_PER_INCH = HardwareConstants.DRIVETRAIN_DRIVE_GEAR_RATIO / HardwareConstants.DRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = HardwareConstants.DRIVETRAIN_DRIVE_TICK_DISTANCE; // converts rotations/sec into inches per second.
    public static final double DRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY = HardwareConstants.DRIVETRAIN_DRIVE_TICKS_PER_INCH; // converts inches per second into rotations/sec

    public static final double DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 35" side-to-side with bumpers
    public static final double DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE = 22.75; // (in inches) 38" front-to-back with bumpers
    public static final double DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
    public static final double DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
}