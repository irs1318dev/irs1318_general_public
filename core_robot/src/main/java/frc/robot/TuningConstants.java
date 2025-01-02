package frc.robot;

import java.util.List;
import java.util.stream.Collectors;

/**
 * All constants related to tuning the operation of the robot.
 *
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = false;
    public static final boolean USE_ADVANTAGE_KIT = true;
    public static final boolean LOG_NULL_WHILE_DISABLED = true;
    public static final boolean RETREIVE_PDH_FIRST = true;

    public static boolean THROW_EXCEPTIONS = true;
    public static boolean LOG_EXCEPTIONS = true;
    public static double LOOP_DURATION = 0.02; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)
    public static int LOOPS_PER_SECOND = 50; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;
    public static final boolean PERFORM_COSTLY_TASKS_WHILE_DISABLED = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double ZERO = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2024;
    public static final boolean LOG_TO_FILE = false; // TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = false;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;
    public static final boolean USE_LOGGING_FREQUENCY = true; // TuningConstants.COMPETITION_ROBOT;
    public static final int DEFAULT_LOGGING_FREQUENCY = 10; // number of entries to ignore between logging

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
    public static final double POWER_OVERCURRENT_TRACKING_MAX_VALUE = 1000.0; // 1000 amos is an unrealistic max value to use for overcurrent
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 140.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 180.0;

    //================================================= Macros/Vision ======================================================

    public static final double VISION_ODOMETRY_ACCURACY_TRESHOLD_RANGE = 30;
    public static final boolean SDSDRIVETRAIN_USE_VISION = true;

    //=========================================== 2024 AprilTag Location guide ==============================================
    //// | TAG                                 |  ID  |    X    |    Y    |   Z    | THETA |
    //// |-------------------------------------|------|---------|---------|--------|-------|
    //// | APRILTAG_BLUE_SOURCE_RIGHT_ID       |   1  |  268.07 |    6.26 |  53.38 | 120.0 |
    //// | APRILTAG_BLUE_SOURCE_LEFT_ID        |   2  |  311.59 |   31.37 |  53.38 | 120.0 |
    //// | APRILTAG_RED_SPEAKER_OFFCENTER_ID   |   3  |  327.12 |  192.75 |  57.13 | 180.0 |
    //// | APRILTAG_RED_SPEAKER_CENTER_ID      |   4  |  327.12 |  215.0  |  57.13 | 180.0 |
    //// | APRILTAG_RED_AMP_ID                 |   5  |  253.16 |  319.58 |  53.38 | 270.0 |
    //// | APRILTAG_BLUE_AMP_ID                |   6  | -253.16 |  319.58 |  53.38 | 270.0 |
    //// | APRILTAG_BLUE_SPEAKER_CENTER_ID     |   7  | -327.12 |  215.0  |  57.13 |   0.0 |
    //// | APRILTAG_BLUE_SPEAKER_OFFCENTER_ID  |   8  | -327.12 |  192.75 |  57.13 |   0.0 |
    //// | ARPILTAG_RED_SOURCE_RIGHT_ID        |   9  | -311.59 |   31.37 |  53.38 |  60.0 |
    //// | APRILTAG_RED_SOURCE_LEFT_ID         |  10  | -268.07 |    6.26 |  53.38 |  60.0 |
    //// | APRILTAG_RED_STAGE_LEFT_ID          |  11  |  143.0  |  142.77 |  52.0  | 300.0 |
    //// | APRILTAG_RED_STAGE_RIGHT_ID         |  12  |  143.0  |  173.68 |  52.0  |  60.0 |
    //// | APRILTAG_RED_CENTER_STAGE_ID        |  13  |  116.13 |  158.5  |  52.0  | 180.0 |
    //// | APRILTAG_BLUE_CENTER_STAGE_ID       |  14  | -116.13 |  158.5  |  52.0  |   0.0 |
    //// | APRILTAG_BLUE_STAGE_LEFT_ID         |  15  | -143.0  |  173.68 |  52.0  | 120.0 |
    //// | APRILTAG_BLUE_STAGE_RIGHT_ID        |  16  | -143.0  |  142.77 |  52.0  | 240.0 |
    ////
    //// Conversion from FIRST's published values: (x - 325.615, y - ~3.42, z, rot)
    public static final double APRILTAG_RED_SPEAKER_X_POSITION = 327.12;
    public static final double APRILTAG_RED_SPEAKER_Y_POSITION = 215.0;
    public static final double APRILTAG_BLUE_SPEAKER_X_POSITION = -327.12;
    public static final double APRILTAG_BLUE_SPEAKER_Y_POSITION = 215.0;

    public static final int APRILTAG_BLUE_SOURCE_RIGHT_ID = 1;
    public static final int APRILTAG_BLUE_SOURCE_LEFT_ID = 2;
    public static final int APRILTAG_RED_SPEAKER_OFFCENTER_ID = 3;
    public static final int APRILTAG_RED_SPEAKER_CENTER_ID = 4;
    public static final int APRILTAG_RED_AMP_ID = 5;
    public static final int APRILTAG_BLUE_AMP_ID = 6;
    public static final int APRILTAG_BLUE_SPEAKER_CENTER_ID = 7;
    public static final int APRILTAG_BLUE_SPEAKER_OFFCENTER_ID = 8;
    public static final int APRILTAG_RED_SOURCE_RIGHT_ID = 9;
    public static final int APRILTAG_RED_SOURCE_LEFT_ID = 10;
    public static final int APRILTAG_RED_STAGE_LEFT_ID = 11;
    public static final int APRILTAG_RED_STAGE_RIGHT_ID = 12;
    public static final int APRILTAG_RED_CENTER_STAGE_ID = 13;
    public static final int APRILTAG_BLUE_CENTER_STAGE_ID = 14;
    public static final int APRILTAG_BLUE_STAGE_LEFT_ID = 15;
    public static final int APRILTAG_BLUE_STAGE_RIGHT_ID = 16;

    public static final List<Integer> VISION_SPEAKER_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_SPEAKER_CENTER_ID, TuningConstants.APRILTAG_BLUE_SPEAKER_OFFCENTER_ID);
    public static final String VISION_SPEAKER_BLUE_STRING = TuningConstants.VISION_SPEAKER_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_SPEAKER_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_SPEAKER_CENTER_ID, TuningConstants.APRILTAG_RED_SPEAKER_OFFCENTER_ID);
    public static final String VISION_SPEAKER_RED_STRING = TuningConstants.VISION_SPEAKER_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    public static final List<Integer> VISION_STAGE_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_STAGE_LEFT_ID, TuningConstants.APRILTAG_BLUE_CENTER_STAGE_ID, TuningConstants.APRILTAG_BLUE_STAGE_RIGHT_ID);
    public static final String VISION_STAGE_BLUE_STRING = TuningConstants.VISION_STAGE_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_STAGE_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_STAGE_LEFT_ID, TuningConstants.APRILTAG_RED_CENTER_STAGE_ID, TuningConstants.APRILTAG_RED_STAGE_RIGHT_ID);
    public static final String VISION_STAGE_RED_STRING = TuningConstants.VISION_STAGE_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    public static final List<Integer> VISION_AMP_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_AMP_ID);
    public static final String VISION_AMP_BLUE_STRING = TuningConstants.VISION_AMP_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_AMP_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_AMP_ID);
    public static final String VISION_AMP_RED_STRING = TuningConstants.VISION_AMP_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Finding AprilTags to determine if theres enough valid data to translate
    public static final int TAGS_MISSED_THRESHOLD = 30;
    public static final int TAGS_FOUND_THRESHOLD = 5;
    public static final double ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE = 1.0; // in inches

    // Acceptable vision centering range values in degrees
    public static final double MAX_PID_TURNING_RANGE_DEGREES = 2.0;

    // How long the robot system must remain centered on the target when using time
    public static final double PID_TURNING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;
    public static final double MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE = 0.7;

    // Acceptable vision distance from tape in angles
    public static final double MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR = 4.0;

    // PID settings for Centering the robot on a vision target from one stationary place, based on a single sample
    public static final double STATIONARY_SINGLE_TURNING_PID_KP = 0.02;
    public static final double STATIONARY_SINGLE_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_KD = 0.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_SINGLE_TURNING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target from one stationary place, based on continuous samples
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KP = 0.02;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KD = 0.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_MAX = 0.4;

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

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 50;

    public static final double ORIENTATION_TURN_THRESHOLD = 2.0; // number of degrees off at which point we give up trying to face an angle

    //================================================== Driver Feedback ========================================================

    public static final double ENDGAME_RUMBLE = 20.0;

    public static final int LED_START = 0;
    public static final int LED_COUNT = 8;
    public static final int LED_STRIP_LED_START = TuningConstants.LED_COUNT;
    public static final int LED_STRIP_LED_COUNT = 60; // 60 LEDs per meter-long strip from CTRE
    public static final int TOTAL_NUMBER_LEDS = TuningConstants.LED_COUNT + TuningConstants.LED_STRIP_LED_COUNT;

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

    //================================================== SDS DriveTrain ==============================================================

    // Drivetrain PID keys/default values:
    public static final boolean TANK_DRIVETRAIN_USE_PID = false;
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
    public static final double TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY = 100.0;
    public static final double TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION = 200.0;

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
    public static final double TANK_DRIVETRAIN_DEAD_ZONE_VELOCITY_X = .05;
    public static final double TANK_DRIVETRAIN_DEAD_ZONE_VELOCITY_Y = .05;
    public static final double TANK_DRIVETRAIN_EXPONENTIAL = 1.0;
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

    //================================================== Intake ==============================================================

    public static final double INTAKE_IN_POWER_LEVEL = -0.9;
    public static final double INTAKE_OUT_POWER_LEVEL = 0.9;

    //================================================== Stinger ==============================================================

    public static final double STINGER_MAX_VELOCTIY = 0.5;
    public static final double STINGER_SLOW_BACK_VELOCTIY = -0.275;

    //================================================== Shooter ==============================================================

    public static final boolean SHOOTER_SCALE_BASED_ON_VOLTAGE = false;
    public static final double SHOOTER_VELOCITY_TUNING_VOLTAGE = 12.5;

    public static final double SHOOTER_MAX_POWER_LEVEL = 1.0;

    public static final double SHOOTER_VELOCITY_PID_KP_DEFAULT = 0.003;
    public static final double SHOOTER_VELOCITY_PID_KI_DEFAULT = 0.0;
    public static final double SHOOTER_VELOCITY_PID_KD_DEFAULT = 0.0;
    public static final double SHOOTER_VELOCITY_PID_KF_DEFAULT = 1.0;
    public static final double SHOOTER_VELOCITY_PID_KS_DEFAULT = 2000.0;

    public static final double SHOOTER_CLOSE_SHOT_VELOCITY = 0.7;
    public static final double SHOOTER_MIDDLE_SHOT_VELOCITY = 0.52;
    public static final double SHOOTER_FAR_SHOT_VELOCITY = 0.7;

    public static final double SHOOTER_LOWER_KICKER_DURATION = 0.5;
    public static final double SHOOTER_SPIN_UP_DURATION = 1.75;
    public static final double SHOOTER_FIRE_DURATION = 0.75;

    public static final double SHOOTER_REVERSE_DURATION = 0.5;

    public static final double SHOOTER_DEVIANCE = 0.025;
    public static final double SHOOTER_MAX_COUNTER_RATE = 2000;

    public static final double SHOOTER_TARGETING_LIGHT_ACTIVATION_THRESHOLD = 0.5;
}