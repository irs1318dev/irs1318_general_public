package frc.robot;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
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
    public static final double PERRY_THE_PLATYPUS = 0.0;
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

    public static final boolean CANCEL_AUTONOMOUS_ROUTINE_ON_DISABLE = true;
    public static final boolean TRAJECTORY_FORCE_BUILD = false;

    //================================================= Power ======================================================

    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_TRACKING_MAX_VALUE = 1000.0; // 1000 amos is an unrealistic max value to use for overcurrent
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 140.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 180.0;

    //================================================= Vision ======================================================

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;

    // Acceptable vision distance from tape in inches
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 3.25;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double VISION_STATIONARY_CENTERING_PID_KP = 0.0125;
    public static final double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KD = 0.01;
    public static final double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static final double VISION_STATIONARY_CENTERING_PID_MIN = -0.4;
    public static final double VISION_STATIONARY_CENTERING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target
    public static final double VISION_MOVING_CENTERING_PID_KP = 0.02;
    public static final double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KD = 0.03;
    public static final double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static final double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static final double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static final double VISION_ADVANCING_PID_KP = 0.01;
    public static final double VISION_ADVANCING_PID_KI = 0.0;
    public static final double VISION_ADVANCING_PID_KD = 0.0;
    public static final double VISION_ADVANCING_PID_KF = 0.0;
    public static final double VISION_ADVANCING_PID_KS = 1.0;
    public static final double VISION_ADVANCING_PID_MIN = -0.3;
    public static final double VISION_ADVANCING_PID_MAX = 0.3;

    // PID settings for Advancing the robot quickly towards a vision target
    public static final double VISION_FAST_ADVANCING_PID_KP = 0.01;
    public static final double VISION_FAST_ADVANCING_PID_KI = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KD = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KF = 0.0;
    public static final double VISION_FAST_ADVANCING_PID_KS = 1.0;
    public static final double VISION_FAST_ADVANCING_PID_MIN = -0.45;
    public static final double VISION_FAST_ADVANCING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 10000;

    //================================================== Indicator Lights ========================================================

    public static final double INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE = 3.0;
    public static final double ENDGAME_RUMBLE = 10.0;

    //================================================== DriveTrain ==============================================================

    public static final boolean DRIVETRAIN_USE_PID = true;
    public static final boolean DRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean DRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean DRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = false;

    //public static final double[] DRIVETRAIN_STEER_MOTOR_ABSOLUTE_OFFSET = new double[] { -135.0, 65.0, -96.0, -14.0 };
    public static final double DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = -38.15;
    public static final double DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = -115.93;
    public static final double DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = 118.08;
    public static final double DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = 177.37;

    // Position PID (angle) per-module
    public static final double DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR1_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR2_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR3_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_STEER_MOTOR4_POSITION_PID_KF = 0.0;

    // Velocity PID (drive) per-module
    public static final double DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = 17000.0; // 21400 was highest speed at full throttle FF on blocks. this is #ticks / 100ms

    public static final double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KP = 0.09;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KP = 0.09;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KP = 0.09;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KP = 0.09;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_VELOCITY_PID_KF = 0.0478; // .0478 ==> ~ 1023 / 21400 (100% control authority)

    public static final double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR1_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR2_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR3_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KP = 1.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_DRIVE_MOTOR4_POSITION_PID_KF = 0.0;

    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double DRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double DRIVETRAIN_OMEGA_MAX_OUTPUT = 4.0;
    public static final double DRIVETRAIN_OMEGA_MIN_OUTPUT = -4.0;

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

    public static final boolean DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 30.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 40.0;
    public static final double DRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 100.0;

    public static final boolean DRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double DRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 11.0;
    public static final boolean DRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double DRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 30.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 40.0;
    public static final double DRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 100.0;

    public static final boolean DRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double DRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.25;

    public static final double DRIVETRAIN_DEAD_ZONE_TURN = 0.20;
    public static final double DRIVETRAIN_DEAD_ZONE_VELOCITY = 0.15;
    public static final double DRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.15;

    public static final double DRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double DRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    // REDUCED FOR OUTREACH
    public static final double DRIVETRAIN_MAX_VELOCITY = TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * 0.9; // max velocity in inches per second
    public static final double DRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.DRIVETRAIN_MAX_VELOCITY;
    public static final double DRIVETRAIN_TURN_GOAL_VELOCITY = 10.0 * 0.25; // degrees per second for turn goal // REDUCED FOR OUTREACH
    public static final double DRIVETRAIN_TURN_SCALE = 4.0; // radians per second
    public static final double DRIVETRAIN_TURN_APPROXIMATION = 0.5; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double DRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double DRIVETRAIN_MAX_PATH_TURN_VELOCITY = 45.0; // in degrees per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.90 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.85 * TuningConstants.DRIVETRAIN_MAX_VELOCITY; // in inches per second per second

    //================================================== Intake ==============================================================

    public static final double POWERCELL_ROLLER_MOTOR_INTAKE_POWER = 0.5;
    public static final double POWERCELL_ROLLER_MOTOR_OUTTAKE_POWER = -0.5;

    public static final double POWERCELL_KICKER_MOTOR_FEED_POWER = 0.35;
    public static final double POWERCELL_KICKER_MOTOR_FEED_REVERSE_POWER = -0.5;

    public static final double POWERCELL_FLYWHEEL_REVERSE_POWER = -0.4;

    public static final double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KP = 0.55;
    public static final double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KI = 0.0;
    public static final double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KD = 0.75;
    public static final double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KF = 0.0113666; // 1023 / 90000
    public static final double POWERCELL_FLYWHEEL_ONE_VELOCITY_PID_KS = 85000.0;

    public static final double POWERCELL_FLYWHEEL_MIN_PERCENTILE = 0.24;
    public static final double POWERCELL_FLYWHEEL_SHOOT_PERCENTILE = 0.11; // .11 + .24 ==> .35
    public static final double POWERCELL_FLYWHEEL_PERCENTILE_MULTIPLIER = 0.26; // .26 + .24 ==> .5

    public static final int POWERCELL_FLYWHEEL_VELOCITY_PERIOD = 10;
    public static final int POWERCELL_FLYWHEEL_VELOCITY_WINDOWSIZE = 32;
    public static final boolean POWERCELL_FLYWHEEL_MASTER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final boolean POWERCELL_FLYWHEEL_FOLLOWER_VELOCITY_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double POWERCELL_FLYWHEEL_MASTER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE = 12.0;
    public static final double POWERCELL_FLYWHEEL_FOLLOWER_VELOCITY_VOLTAGE_COMPENSATION_MAXVOLTAGE = 12.0;

    public static final double POWERCELL_THROUGHBEAM_CUTOFF = 2.7;
    public static final double POWERCELL_CAROUSEL_COUNT_THRESHOLD = 5.0;
    public static final double POWERCELL_CAROUSEL_MOTOR_POWER_INDEXING = -0.4;
    public static final double POWERCELL_CAROUSEL_MOTOR_POWER_SHOOTING = 0.4;
    public static final double POWERCELL_CAROUSEL_MECHANISM_INDEXING_TIMEOUT = 2.0;
    public static final double POWERCELL_FLYWHEEL_ALLOWABLE_ERROR_RANGE = 500; // ticks per 100ms

    public static final boolean POWERCELL_CAROUSEL_USE_PID = true;

    public static final double POWERCELL_CAROUSEL_PID_KP = 0.0002;
    public static final double POWERCELL_CAROUSEL_PID_KI = 0.0;
    public static final double POWERCELL_CAROUSEL_PID_KD = 0.0;
    public static final double POWERCELL_CAROUSEL_PID_KF = 1.0;
    public static final double POWERCELL_CAROUSEL_PID_KS = 3200.0;
    public static final double POWERCELL_CAROUSEL_MAX_POWER = 1.0;
}
