package frc.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = true;
    public static boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;
    public static boolean LOG_EXCEPTIONS = true;

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double PERRY_THE_PLATYPUS = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2022;
    public static final boolean LOG_TO_FILE = TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = true;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;

    //================================================== Autonomous ==============================================================

    public static final boolean CANCEL_AUTONOMOUS_ROUTINE_ON_DISABLE = true;

    public static final double DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA = 1.0;

    // Navx Turn Constants
    public static final double MAX_NAVX_TURN_RANGE_DEGREES = 5.0;
    public static final double MAX_NAVX_FAST_TURN_RANGE_DEGREES = 5.0;
    public static final double NAVX_FAST_TURN_TIMEOUT = 1.25;
    public static final double NAVX_TURN_COMPLETE_TIME = 0.4;
    public static final double NAVX_TURN_COMPLETE_CURRENT_VELOCITY_DELTA = 0;
    public static final double NAVX_TURN_COMPLETE_DESIRED_VELOCITY_DELTA = 0;

    // Navx Turn PID Constants
    public static final double NAVX_TURN_PID_KP = 0.025; // 0.04
    public static final double NAVX_TURN_PID_KI = 0.0;
    public static final double NAVX_TURN_PID_KD = 0.02;
    public static final double NAVX_TURN_PID_KF = 0.0;
    public static final double NAVX_TURN_PID_KS = 1.0;
    public static final double NAVX_TURN_PID_MIN = -0.8;
    public static final double NAVX_TURN_PID_MAX = 0.8;
    public static final double NAVX_FAST_TURN_PID_KP = 0.01;
    public static final double NAVX_FAST_TURN_PID_KI = 0.0;
    public static final double NAVX_FAST_TURN_PID_KD = 0.0;
    public static final double NAVX_FAST_TURN_PID_KF = 0.0;
    public static final double NAVX_FAST_TURN_PID_KS = 1.0;
    public static final double NAVX_FAST_TURN_PID_MIN = -0.8;
    public static final double NAVX_FAST_TURN_PID_MAX = 0.8;

    //================================================= Vision ======================================================

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;

    // How long the robot system must remain centered on the target when using time
    public static final double VISION_CENTERING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double VISION_STATIONARY_CENTERING_PID_KP = 0.025;
    public static final double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KD = 0.01;
    public static final double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static final double VISION_STATIONARY_CENTERING_PID_MIN = -0.4;
    public static final double VISION_STATIONARY_CENTERING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target
    public static final double VISION_MOVING_CENTERING_PID_KP = 0.01;
    public static final double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KD = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static final double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static final double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static final double VISION_ADVANCING_PID_KP = 0.015;
    public static final double VISION_ADVANCING_PID_KI = 0.0;
    public static final double VISION_ADVANCING_PID_KD = 0.0;
    public static final double VISION_ADVANCING_PID_KF = 0.0;
    public static final double VISION_ADVANCING_PID_KS = 1.0;
    public static final double VISION_ADVANCING_PID_MIN = -0.3;
    public static final double VISION_ADVANCING_PID_MAX = 0.3;

    // PID settings for Advancing the robot quickly towards a vision target
    public static final double VISION_FAST_ADVANCING_PID_KP = 0.15;
    public static final double VISION_FAST_ADVANCING_PID_KI = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KD = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KF = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KS = 1.0;
    public static final double VISION_FAST_ADVANCING_PID_MIN = -0.45;
    public static final double VISION_FAST_ADVANCING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 500;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    public static final int CANDLE_LED_COUNT = 8;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int CANDLE_TOTAL_NUMBER_LEDS = TuningConstants.CANDLE_LED_COUNT; //+ TuningConstants.LED_STRIP_LED_COUNT; // * 2;

    public static final int CANDLE_ANIMATION_SLOT_1 = 0;
    public static final int CANDLE_ANIMATION_SLOT_2 = 1;

    // IRS1318 Purple color
    public static final int INDICATOR_PURPLE_RED = 101;
    public static final int INDICATOR_PURPLE_GREEN = 34;
    public static final int INDICATOR_PURPLE_BLUE = 129;
    public static final int INDICATOR_PURPLE_WHITE = 0;

    // No color
    public static final int INDICATOR_OFF_COLOR_RED = 0;
    public static final int INDICATOR_OFF_COLOR_GREEN = 0;
    public static final int INDICATOR_OFF_COLOR_BLUE = 0;
    public static final int INDICATOR_OFF_COLOR_WHITE = 0;

    // Bright Red color
    public static final int INDICATOR_RED_COLOR_RED = 255;
    public static final int INDICATOR_RED_COLOR_GREEN = 0;
    public static final int INDICATOR_RED_COLOR_BLUE = 0;
    public static final int INDICATOR_RED_COLOR_WHITE = 0;

    // Bright Yellow color
    public static final int INDICATOR_YELLOW_COLOR_RED = 255;
    public static final int INDICATOR_YELLOW_COLOR_GREEN = 255;
    public static final int INDICATOR_YELLOW_COLOR_BLUE = 0;
    public static final int INDICATOR_YELLOW_COLOR_WHITE = 0;

    // Bright Green color
    public static final int INDICATOR_GREEN_COLOR_RED = 0;
    public static final int INDICATOR_GREEN_COLOR_GREEN = 255;
    public static final int INDICATOR_GREEN_COLOR_BLUE = 0;
    public static final int INDICATOR_GREEN_COLOR_WHITE = 0;

    public static final double COMPRESSOR_FILL_RATE = 10.0;
    public static final double COMPRESSOR_ENOUGH_PRESSURE = 110.0;

    //================================================== DriveTrain ==============================================================

    // Drivetrain PID keys/default values:
    public static final boolean DRIVETRAIN_USE_PID = true;
    public static final boolean DRIVETRAIN_USE_CROSS_COUPLING = false;
    public static final boolean DRIVETRAIN_USE_HEADING_CORRECTION = true;

    // Velocity PID (right)
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KP = 0.09;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KS = 17000.0; // 21400 was highest speed at full throttle FF on blocks

    // Velocity PID (left)
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KP = 0.09;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KS = 17000.0; // 21400 was highest speed at full throttle FF on blocks

    // Path PID (right)
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KP = 0.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KF = 0.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KV = 1.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KCC = 0.0;
    public static final double DRIVETRAIN_PATH_RIGHT_HEADING_CORRECTION = 0.0;

    // gets the max speed in inches per second
    // (TalonSRX: 10 * (ticks per 100ms) * (inches per tick) * (10) == in / s)
    // (SparkMAX: (rotations per second) * (inches per rotation) == in / s)
    public static final double DRIVETRAIN_PATH_RIGHT_MAX_VELOCITY_INCHES_PER_SECOND = 10.0 * TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS * HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE;
    // public static final double DRIVETRAIN_PATH_RIGHT_MAX_VELOCITY_INCHES_PER_SECOND = TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS * HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE;

    // Path PID (left)
    public static final double DRIVETRAIN_PATH_PID_LEFT_KP = 0.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KF = 0.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KV = 1.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KCC = 0.0;
    public static final double DRIVETRAIN_PATH_LEFT_HEADING_CORRECTION = 0.0;

    // gets the max control speed in inches per second
    // (TalonSRX: 10 * (ticks per 100ms) * (inches per tick) * (10) == in / s)
    public static final double DRIVETRAIN_PATH_LEFT_MAX_VELOCITY_INCHES_PER_SECOND = 10.0 * TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS * HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE;
    // public static final double DRIVETRAIN_PATH_LEFT_MAX_VELOCITY_INCHES_PER_SECOND = TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS * HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE;

    // Position PID (right)
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KP = 0.0002;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KF = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KCC = 0.0001;

    // Position PID (left)
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KP = 0.0002;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KF = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KCC = 0.0001;

    // Brake PID (right)
    public static final double DRIVETRAIN_BRAKE_PID_RIGHT_KP = 0.0004;
    public static final double DRIVETRAIN_BRAKE_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_BRAKE_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_BRAKE_PID_RIGHT_KF = 0.0;

    // Brake PID (left)
    public static final double DRIVETRAIN_BRAKE_PID_LEFT_KP = 0.0004;
    public static final double DRIVETRAIN_BRAKE_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_BRAKE_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_BRAKE_PID_LEFT_KF = 0.0;

    // Drivetrain choices for one-stick drive
    public static final double DRIVETRAIN_K1 = 1.4;
    public static final double DRIVETRAIN_K2 = 0.5;

    // Drivetrain deadzone/max power levels
    public static final boolean DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_SUPPLY_CURRENT_MAX = 40.0;
    public static final double DRIVETRAIN_SUPPLY_TRIGGER_CURRENT = 40.0;
    public static final double DRIVETRAIN_SUPPLY_TRIGGER_DURATION = 0.1;
    public static final double DRIVETRAIN_X_DEAD_ZONE = .05;
    public static final double DRIVETRAIN_Y_DEAD_ZONE = .05;
    public static final double DRIVETRAIN_MAX_POWER_LEVEL = 1.0;// max power level (velocity)
    public static final double DRIVETRAIN_LEFT_POSITIONAL_NON_PID_MULTIPLICAND = HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE / 60.0;
    public static final double DRIVETRAIN_RIGHT_POSITIONAL_NON_PID_MULTIPLICAND = HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE / 60.0;
    public static final double DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID = 0.2;// max power level (positional, non-PID)

    public static final double DRIVETRAIN_CROSS_COUPLING_ZERO_ERROR_RANGE = 100.0; // (in ticks)
    public static final double DRIVETRAIN_PATH_MAX_POWER_LEVEL = 1.0;
    public static final double DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL = 0.90; // 0.85
    public static final double DRIVETRAIN_BRAKE_MAX_POWER_LEVEL = 0.6;
    public static final double DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL = 1.0;

    public static final boolean DRIVETRAIN_REGULAR_MODE_SQUARING = false;
    public static final boolean DRIVETRAIN_SIMPLE_MODE_SQUARING = false;

    public static final double DRIVETRAIN_ENCODER_ODOMETRY_ANGLE_CORRECTION = 1.0; // account for turning weirdness (any degree offset in the angle)

    //================================================== One-Motor ==============================================================

    public static final boolean ONEMOTOR_USE_PID = false;
    public static final boolean ONEMOTOR_PID_POSITIONAL = false;
    public static final boolean ONEMOTOR_PID_POSITIONAL_MM = false;
    public static final boolean ONEMOTOR_HAS_FOLLOWER = true;

    public static final double ONEMOTOR_PID_KP = 0.5 / 40.0; // 0.1 / 6000.0;
    public static final double ONEMOTOR_PID_KI = 0.0;
    public static final double ONEMOTOR_PID_KD = 0.0;
    public static final double ONEMOTOR_PID_KF = 0.0; // 1.0 / 6000.0;
    public static final double ONEMOTOR_PID_MIN_OUTPUT = -0.1; // 1.0 / 6000.0;
    public static final double ONEMOTOR_PID_MAX_OUTPUT = 0.1; // 1.0 / 6000.0;
    public static final int ONEMOTOR_PID_MM_CRUISE_VELOC = 0;
    public static final int ONEMOTOR_PID_MM_ACCEL = 0;

    public static final double ONEMOTOR_PID_MAX_POSITION = 25.0; // (SPARK MAX NEO: in rotations) (Talon SRX: in ticks. 36 inches / (4.75 inches/rotation) * (4096 ticks/rotation))
    public static final double ONEMOTOR_PID_MAX_VELOCITY = 6000.0;

    public static final boolean ONEMOTOR_INVERT_OUTPUT = false;
    public static final boolean ONEMOTOR_INVERT_SENSOR = false;

    public static final boolean FOLLOWER_INVERT_OUTPUT = true;

    public static final boolean ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED = false;
    public static final boolean ONEMOTOR_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN = true;
    public static final boolean ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED = false;
    public static final boolean ONEMOTOR_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN = true;
}
