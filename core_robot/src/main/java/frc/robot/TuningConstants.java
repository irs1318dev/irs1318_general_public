package frc.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = true;
    public static boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;
    public static boolean LOG_EXCEPTIONS = true;
    public static double LOOP_DURATION = 0.02; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)
    public static int LOOPS_PER_SECOND = 50; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double ZERO = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2023;
    public static final boolean LOG_TO_FILE = true; // TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = false;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;

    //================================================== Autonomous ==============================================================

    public static final boolean TRAJECTORY_FORCE_BUILD = false;

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

    //================================================= Power ======================================================

    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 140.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 180.0;

    //================================================= Vision ======================================================

    // Finding AprilTags to determine if theres enough valid data to translate 
    public static final int TAGS_MISSED_THRESHOLD = 30;
    public static final int TAGS_FOUND_THRESHOLD = 5;
    public static final double ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE = 1.0; // in inches

    // Acceptable vision centering range values in degrees
    public static final double MAX_PID_TURNING_RANGE_DEGREES = 7.0;

    // How long the robot system must remain centered on the target when using time
    public static final double PID_TURNING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;
    public static final double MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE = 0.7;

    // Acceptable vision distance from tape in angles 
    public static final double MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR = 4.0;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double STATIONARY_PID_TURNING_PID_KP = 0.027;
    public static final double STATIONARY_PID_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_PID_TURNING_PID_KD = 0.01;
    public static final double STATIONARY_PID_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_PID_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_PID_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_PID_TURNING_PID_MAX = 0.4;

    // PID settings for rotating the robot based on a vision target while in-motion
    public static final double VISION_MOVING_TURNING_PID_KP = 0.012;
    public static final double VISION_MOVING_TURNING_PID_KI = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KD = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KF = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KS = 1.0;
    public static final double VISION_MOVING_TURNING_PID_MIN = -0.3;
    public static final double VISION_MOVING_TURNING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_MOVING_PID_KP = 0.015;
    public static final double VISION_MOVING_PID_KI = 0.0;
    public static final double VISION_MOVING_PID_KD = 0.0;
    public static final double VISION_MOVING_PID_KF = 0.0;
    public static final double VISION_MOVING_PID_KS = 1.0;
    public static final double VISION_MOVING_PID_MIN = -0.3;
    public static final double VISION_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_X_PID_KP = 0.023;
    public static final double VISION_AT_TRANSLATION_X_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_X_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_X_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_Y_PID_KP = 0.023;
    public static final double VISION_AT_TRANSLATION_Y_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_Y_PID_MAX = 0.3;

    // PID settings for translating the robot slowly based on a vision target
    public static final double VISION_SLOW_MOVING_PID_KP = 0.013;
    public static final double VISION_SLOW_MOVING_PID_KI = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KD = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KF = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KS = 1.0;
    public static final double VISION_SLOW_MOVING_PID_MIN = -0.3;
    public static final double VISION_SLOW_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot quickly based on a vision target
    public static final double VISION_FAST_MOVING_PID_KP = 0.17;
    public static final double VISION_FAST_MOVING_PID_KI = 0.0;
    public static final double VISION_FAST_MOVING_PID_KD = 0.0;
    public static final double VISION_FAST_MOVING_PID_KF = 0.0;
    public static final double VISION_FAST_MOVING_PID_KS = 1.0;
    public static final double VISION_FAST_MOVING_PID_MIN = -0.45;
    public static final double VISION_FAST_MOVING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 500;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;

    public static final int CANDLE_LED_START = 0;
    public static final int CANDLE_LED_COUNT = 8;
    public static final int LED_STRIP_LED_START = TuningConstants.CANDLE_LED_COUNT;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int CANDLE_TOTAL_NUMBER_LEDS = TuningConstants.CANDLE_LED_COUNT + TuningConstants.LED_STRIP_LED_COUNT;

    public static final int CANDLE_ANIMATION_SLOT_0 = 0;
    public static final int CANDLE_ANIMATION_SLOT_1 = 1;
    public static final int CANDLE_ANIMATION_SLOT_2 = 2;
    public static final int CANDLE_ANIMATION_SLOT_3 = 3;

    // IRS1318 Purple color
    public static final int INDICATOR_PURPLE_COLOR_RED = 101;
    public static final int INDICATOR_PURPLE_COLOR_GREEN = 34;
    public static final int INDICATOR_PURPLE_COLOR_BLUE = 129;
    public static final int INDICATOR_PURPLE_COLOR_WHITE = 0;

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

    // Bright Red color
    public static final int INDICATOR_RED_COLOR_RED = 255;
    public static final int INDICATOR_RED_COLOR_GREEN = 0;
    public static final int INDICATOR_RED_COLOR_BLUE = 0;
    public static final int INDICATOR_RED_COLOR_WHITE = 0;

    // Blue
    public static final int INDICATOR_BLUE_COLOR_RED = 0;
    public static final int INDICATOR_BLUE_COLOR_GREEN = 0;
    public static final int INDICATOR_BLUE_COLOR_BLUE = 255;
    public static final int INDICATOR_BLUE_COLOR_WHITE = 0;

    // Orange
    public static final int INDICATOR_ORANGE_COLOR_RED = 255;
    public static final int INDICATOR_ORANGE_COLOR_GREEN = 165;
    public static final int INDICATOR_ORANGE_COLOR_BLUE = 0;
    public static final int INDICATOR_ORANGE_COLOR_WHITE = 0;

    // Rainbow
    public static final int INDICATOR_RAINBOW_BRIGHTNESS = 1;
    public static final double INDICATOR_RAINBOW_SPEED = 0.25;
    public static final boolean INDICATOR_RAINBOW_REVERSE_DIRECTION = false;

    // No color
    public static final int INDICATOR_OFF_COLOR_RED = 0;
    public static final int INDICATOR_OFF_COLOR_GREEN = 0;
    public static final int INDICATOR_OFF_COLOR_BLUE = 0;
    public static final int INDICATOR_OFF_COLOR_WHITE = 0;

    //================================================== DriveTrain ==============================================================

    // Drivetrain PID keys/default values:
    public static final boolean TANK_DRIVETRAIN_USE_PID = true;
    public static final boolean TANK_DRIVETRAIN_USE_CROSS_COUPLING = false;
    public static final boolean TANK_DRIVETRAIN_USE_HEADING_CORRECTION = true;

    // Velocity PID (right)
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KP = 0.09;
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KI = 0.0;
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KD = 0.0;
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KS = 17000.0; // 21400 was highest speed at full throttle FF on blocks

    // Velocity PID (left)
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KP = 0.09;
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KI = 0.0;
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KD = 0.0;
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)
    public static final double TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KS = 17000.0; // 21400 was highest speed at full throttle FF on blocks

    // Path PID (right)
    public static final double TANK_DRIVETRAIN_PATH_PID_RIGHT_KP = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_RIGHT_KI = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_RIGHT_KD = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_RIGHT_KF = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_RIGHT_KV = 1.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_RIGHT_KCC = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_RIGHT_HEADING_CORRECTION = 0.0;

    // gets the max speed in inches per second
    // (TalonSRX: 10 * (ticks per 100ms) * (inches per tick) * (10) == in / s)
    // (SparkMAX: (rotations per second) * (inches per rotation) == in / s)
    public static final double TANK_DRIVETRAIN_PATH_LEFT_MAX_VELOCITY_INCHES_PER_SECOND = 10.0 * TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KS * HardwareConstants.TANK_DRIVETRAIN_LEFT_PULSE_DISTANCE;
    public static final double TANK_DRIVETRAIN_PATH_RIGHT_MAX_VELOCITY_INCHES_PER_SECOND = 10.0 * TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KS * HardwareConstants.TANK_DRIVETRAIN_RIGHT_PULSE_DISTANCE;
    public static final double TANK_DRIVETRAIN_PATH_MAX_VELOCITY_INCHES_PER_SECOND = 0.5 * (TuningConstants.TANK_DRIVETRAIN_PATH_LEFT_MAX_VELOCITY_INCHES_PER_SECOND + TuningConstants.TANK_DRIVETRAIN_PATH_RIGHT_MAX_VELOCITY_INCHES_PER_SECOND) / 2.0;
    public static final double TANK_DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 5.0;
    public static final double TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 100.0;
    public static final double TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 200.0;

    // Path PID (left)
    public static final double TANK_DRIVETRAIN_PATH_PID_LEFT_KP = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_LEFT_KI = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_LEFT_KD = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_LEFT_KF = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_LEFT_KV = 1.0;
    public static final double TANK_DRIVETRAIN_PATH_PID_LEFT_KCC = 0.0;
    public static final double TANK_DRIVETRAIN_PATH_LEFT_HEADING_CORRECTION = 0.0;

    // Position PID (right)
    public static final double TANK_DRIVETRAIN_POSITION_PID_RIGHT_KP = 0.0002;
    public static final double TANK_DRIVETRAIN_POSITION_PID_RIGHT_KI = 0.0;
    public static final double TANK_DRIVETRAIN_POSITION_PID_RIGHT_KD = 0.0;
    public static final double TANK_DRIVETRAIN_POSITION_PID_RIGHT_KF = 0.0;
    public static final double TANK_DRIVETRAIN_POSITION_PID_RIGHT_KCC = 0.0001;

    // Position PID (left)
    public static final double TANK_DRIVETRAIN_POSITION_PID_LEFT_KP = 0.0002;
    public static final double TANK_DRIVETRAIN_POSITION_PID_LEFT_KI = 0.0;
    public static final double TANK_DRIVETRAIN_POSITION_PID_LEFT_KD = 0.0;
    public static final double TANK_DRIVETRAIN_POSITION_PID_LEFT_KF = 0.0;
    public static final double TANK_DRIVETRAIN_POSITION_PID_LEFT_KCC = 0.0001;

    // Brake PID (right)
    public static final double TANK_DRIVETRAIN_BRAKE_PID_RIGHT_KP = 0.0004;
    public static final double TANK_DRIVETRAIN_BRAKE_PID_RIGHT_KI = 0.0;
    public static final double TANK_DRIVETRAIN_BRAKE_PID_RIGHT_KD = 0.0;
    public static final double TANK_DRIVETRAIN_BRAKE_PID_RIGHT_KF = 0.0;

    // Brake PID (left)
    public static final double TANK_DRIVETRAIN_BRAKE_PID_LEFT_KP = 0.0004;
    public static final double TANK_DRIVETRAIN_BRAKE_PID_LEFT_KI = 0.0;
    public static final double TANK_DRIVETRAIN_BRAKE_PID_LEFT_KD = 0.0;
    public static final double TANK_DRIVETRAIN_BRAKE_PID_LEFT_KF = 0.0;

    // Drivetrain choices for one-stick drive
    public static final double TANK_DRIVETRAIN_K1 = 1.4;
    public static final double TANK_DRIVETRAIN_K2 = 0.5;

    // Drivetrain deadzone/max power levels
    public static final boolean TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double TANK_DRIVETRAIN_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX = 40.0;
    public static final double TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT = 40.0;
    public static final double TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION = 0.1;
    public static final double TANK_DRIVETRAIN_X_DEAD_ZONE = .05;
    public static final double TANK_DRIVETRAIN_Y_DEAD_ZONE = .05;
    public static final double TANK_DRIVETRAIN_MAX_POWER_LEVEL = 1.0;// max power level (velocity)
    public static final double TANK_DRIVETRAIN_LEFT_POSITIONAL_NON_PID_MULTIPLICAND = HardwareConstants.TANK_DRIVETRAIN_LEFT_PULSE_DISTANCE / 60.0;
    public static final double TANK_DRIVETRAIN_RIGHT_POSITIONAL_NON_PID_MULTIPLICAND = HardwareConstants.TANK_DRIVETRAIN_RIGHT_PULSE_DISTANCE / 60.0;
    public static final double TANK_DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID = 0.2;// max power level (positional, non-PID)

    public static final double TANK_DRIVETRAIN_CROSS_COUPLING_ZERO_ERROR_RANGE = 100.0; // (in ticks)
    public static final double TANK_DRIVETRAIN_PATH_MAX_POWER_LEVEL = 1.0;
    public static final double TANK_DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL = 0.90; // 0.85
    public static final double TANK_DRIVETRAIN_BRAKE_MAX_POWER_LEVEL = 0.6;
    public static final double TANK_DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL = 1.0;

    public static final boolean TANK_DRIVETRAIN_REGULAR_MODE_SQUARING = false;
    public static final boolean TANK_DRIVETRAIN_SIMPLE_MODE_SQUARING = false;

    public static final double TANK_DRIVETRAIN_ENCODER_ODOMETRY_ANGLE_CORRECTION = 1.0; // account for turning weirdness (any degree offset in the angle)

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
