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

    public static final boolean DRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC = true;

    public static final boolean DRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean DRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = true;

    public static final double DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = 128.056;
    public static final double DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = -16.259;
    public static final double DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = 2.725;
    public static final double DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = -30.586;

    public static final boolean DRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING = true;
    public static final double DRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -3.0 * TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE = 3.0 * TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final boolean DRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING = true;
    public static final double DRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.DRIVETRAIN_TURN_SCALE;
    public static final double DRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.DRIVETRAIN_TURN_SCALE;

    // Position PID (angle) per-module
    public static final double DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KP = 0.5;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTORS_MM_PID_KF = 0.34; // 1023 over max speed (3000 ticks per 100ms)
    public static final int DRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC = 48000;
    public static final int DRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL = 48000;

    // Velocity PID (drive) per-module
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = 16000.0; // 20000 was highest speed at full throttle FF on blocks. this is #ticks / 100ms

    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP = 0.1;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF = 0.05115; // .05115 ==> ~ 1023 / 20000 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_OMEGA_MAX_OUTPUT = 5.0;
    public static final double DRIVETRAIN_OMEGA_MIN_OUTPUT = -5.0;

    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double DRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static final double DRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static final boolean DRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT = true;
    public static final double DRIVETRAIN_OVERCURRENT_ADJUSTMENT = 0.75;
    public static final double DRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT = 0.5;

    public static final boolean DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 35.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 35.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 0.25;

    public static final boolean DRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 20.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 30.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 0.1;

    public static final int DRIVETRAIN_SENSOR_FRAME_PERIOD_MS = 10;
    public static final int DRIVETRAIN_PID_FRAME_PERIOD_MS = 100;

    public static final boolean DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.36;

    public static final double DRIVETRAIN_EXPONENTIAL = 2.0;
    public static final double DRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY_X = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY_Y = 0.1;
    public static final double DRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.1;

    public static final double DRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double DRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double DRIVETRAIN_MAX_VELOCITY = TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double DRIVETRAIN_SLOW_MODE_MAX_VELOCITY = 0.3 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // max velocity in inches per second
    public static final double DRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double DRIVETRAIN_TURN_SCALE = 1.6 * Math.PI; // radians per second
    public static final double DRIVETRAIN_SLOW_MODE_TURN_SCALE = 0.3 * TuningConstants.DRIVETRAIN_TURN_SCALE; // radians per second
    public static final double DRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double DRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.60 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double DRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.75 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.75 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double DRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 1.4; // in inches per second
    public static final double DRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 1.25; // in inches per second per second
    public static final double DRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 2.0; // in inches per second
    public static final double DRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 2.0; // in inches per second per second

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
