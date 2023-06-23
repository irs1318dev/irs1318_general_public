package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.*;
import frc.lib.controllers.PIDHandler;
import frc.lib.driver.*;
import frc.lib.filters.*;
import frc.lib.helpers.AnglePair;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.mechanisms.PowerManager.CurrentLimiting;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class DriveTrainMechanismNeo implements IMechanism
{
    private static final int NUM_DRIVE_MODULES = 4;

    private static final int DefaultPidSlotId = 0;
    private static final int MMPidSlotId = 1;

    private static final LoggingKey[] ENCODER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainAbsoluteEncoderAngle1, LoggingKey.DriveTrainAbsoluteEncoderAngle2, LoggingKey.DriveTrainAbsoluteEncoderAngle3, LoggingKey.DriveTrainAbsoluteEncoderAngle4 };
    private static final LoggingKey[] DRIVE_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocity1, LoggingKey.DriveTrainDriveVelocity2, LoggingKey.DriveTrainDriveVelocity3, LoggingKey.DriveTrainDriveVelocity4 };
    private static final LoggingKey[] DRIVE_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainDrivePosition1, LoggingKey.DriveTrainDrivePosition2, LoggingKey.DriveTrainDrivePosition3, LoggingKey.DriveTrainDrivePosition4 };
    // private static final LoggingKey[] DRIVE_ERROR_LOGGING_KEYS = { LoggingKey.DriveTrainDriveError1, LoggingKey.DriveTrainDriveError2, LoggingKey.DriveTrainDriveError3, LoggingKey.DriveTrainDriveError4 };
    private static final LoggingKey[] STEER_VELOCITY_LOGGING_KEYS = { LoggingKey.DriveTrainSteerVelocity1, LoggingKey.DriveTrainSteerVelocity2, LoggingKey.DriveTrainSteerVelocity3, LoggingKey.DriveTrainSteerVelocity4 };
    private static final LoggingKey[] STEER_POSITION_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPosition1, LoggingKey.DriveTrainSteerPosition2, LoggingKey.DriveTrainSteerPosition3, LoggingKey.DriveTrainSteerPosition4 };
    private static final LoggingKey[] STEER_ANGLE_LOGGING_KEYS = { LoggingKey.DriveTrainSteerAngle1, LoggingKey.DriveTrainSteerAngle2, LoggingKey.DriveTrainSteerAngle3, LoggingKey.DriveTrainSteerAngle4 };
    // private static final LoggingKey[] STEER_ERROR_LOGGING_KEYS = { LoggingKey.DriveTrainSteerError1, LoggingKey.DriveTrainSteerError2, LoggingKey.DriveTrainSteerError3, LoggingKey.DriveTrainSteerError4 };
    private static final LoggingKey[] DRIVE_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainDriveVelocityGoal1, LoggingKey.DriveTrainDriveVelocityGoal2, LoggingKey.DriveTrainDriveVelocityGoal3, LoggingKey.DriveTrainDriveVelocityGoal4 };
    private static final LoggingKey[] STEER_GOAL_LOGGING_KEYS = { LoggingKey.DriveTrainSteerPositionGoal1, LoggingKey.DriveTrainSteerPositionGoal2, LoggingKey.DriveTrainSteerPositionGoal3, LoggingKey.DriveTrainSteerPositionGoal4 };

    private static final AnalogOperation[] STEER_SETPOINT_OPERATIONS = new AnalogOperation[] { AnalogOperation.DriveTrainPositionSteer1, AnalogOperation.DriveTrainPositionSteer2, AnalogOperation.DriveTrainPositionSteer3, AnalogOperation.DriveTrainPositionSteer4 };
    private static final AnalogOperation[] DRIVE_SETPOINT_OPERATIONS = new AnalogOperation[] { AnalogOperation.DriveTrainPositionDrive1, AnalogOperation.DriveTrainPositionDrive2, AnalogOperation.DriveTrainPositionDrive3, AnalogOperation.DriveTrainPositionDrive4 };

    // the x offsets of the swerve modules from the default center of rotation
    private final double[] moduleOffsetX;

    // the y offsets of the swerve modules from the default center of rotation
    private final double[] moduleOffsetY;

    private final double[] drivetrainSteerMotorAbsoluteOffsets;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final PigeonManager imuManager;
    private final PowerManager powerManager;

    private final ISparkMax[] steerMotors;
    private final ISparkMax[] driveMotors;
    private final ICANCoder[] absoluteEncoders;

    private final PIDHandler omegaPID;
    private final boolean[] isDirectionSwapped;
    private final PIDHandler pathOmegaPID;
    private final PIDHandler pathXOffsetPID;
    private final PIDHandler pathYOffsetPID;
    private final int[] driveSlotIds;

    private final double[] driveVelocities;
    private final double[] drivePositions;
    private final double[] steerVelocities;
    private final double[] steerPositions;
    private final double[] steerAngles;
    private final double[] encoderAngles;

    private final Setpoint[] result;

    private final SlewRateLimiter xVelocityLimiter;
    private final SlewRateLimiter yVelocityLimiter;
    private final SlewRateLimiter angularVelocityLimiter;

    private boolean firstRun;

    private boolean fieldOriented;
    private boolean maintainOrientation;
    private double desiredYaw;

    private double time;
    private double angle;
    private double xPosition;
    private double yPosition;
    private double deltaT;

    private double robotYaw;

    @Inject
    public DriveTrainMechanismNeo(IDriver driver, LoggingManager logger, IRobotProvider provider, PigeonManager imuManager, PowerManager powerManager, ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.imuManager = imuManager;
        this.powerManager = powerManager;

        this.steerMotors = new ISparkMax[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        this.driveMotors = new ISparkMax[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        this.absoluteEncoders = new ICANCoder[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];

        this.moduleOffsetX =
            new double[]
            {
                -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // Module 1: front-right
                HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // Module 2: front-left
                HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // Module 3: back-left
                -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, // Module 4: back-right
            };

        this.moduleOffsetY =
            new double[]
            {
                -HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // front-right
                -HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // front-left
                HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // back-left
                HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE, // back-right
            };

        this.drivetrainSteerMotorAbsoluteOffsets =
            new double[]
            {
                TuningConstants.DRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET,
                TuningConstants.DRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET,
                TuningConstants.DRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET,
                TuningConstants.DRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET,
            };

        int[] driveMotorCanIds =
            new int[]
            {
                ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_1_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_2_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_3_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_DRIVE_MOTOR_4_CAN_ID
            };

        int[] steerMotorCanIds =
            new int[]
            {
                ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_1_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_2_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_3_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_STEER_MOTOR_4_CAN_ID
            };

        int[] absoluteEncoderCanIds =
            new int[]
            {
                ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_1_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_2_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_3_CAN_ID,
                ElectronicsConstants.DRIVETRAIN_ABSOLUTE_ENCODER_4_CAN_ID
            };

        boolean[] driveMotorInvert =
            new boolean[]
            {
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR1_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR2_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR3_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR4_INVERT_OUTPUT
            };
        
        boolean[] steerMotorInvert =
            new boolean[]
            {
                HardwareConstants.DRIVETRAIN_STEER_MOTOR1_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR2_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR3_INVERT_OUTPUT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR4_INVERT_OUTPUT
            };

        for (int i = 0; i < DriveTrainMechanismNeo.NUM_DRIVE_MODULES; i++)
        {
            this.driveMotors[i] = provider.getSparkMax(driveMotorCanIds[i], SparkMaxMotorType.Brushless);
            this.driveMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.driveMotors[i].setInvertOutput(driveMotorInvert[i]);
            this.driveMotors[i].setPIDF(
                TuningConstants.DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP, 
                TuningConstants.DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI,
                TuningConstants.DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD,
                TuningConstants.DRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF,
                DriveTrainMechanismNeo.DefaultPidSlotId);
            this.driveMotors[i].setPIDF(
                TuningConstants.DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP,
                TuningConstants.DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI,
                TuningConstants.DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD,
                TuningConstants.DRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF,
                DriveTrainMechanismNeo.MMPidSlotId);
            //need to add voltage limiting
            this.driveMotors[i].setControlMode(SparkMaxControlMode.Velocity);
            this.driveMotors[i].setSelectedSlot(DriveTrainMechanismNeo.DefaultPidSlotId);

            this.steerMotors[i] = provider.getSparkMax(steerMotorCanIds[i], SparkMaxMotorType.Brushless);
            this.steerMotors[i].setNeutralMode(MotorNeutralMode.Brake);
            this.steerMotors[i].setInvertOutput(steerMotorInvert[i]);
            this.steerMotors[i].setPIDF(
                TuningConstants.DRIVETRAIN_STEER_MOTORS_POSITION_PID_KP,
                TuningConstants.DRIVETRAIN_STEER_MOTORS_POSITION_PID_KI,
                TuningConstants.DRIVETRAIN_STEER_MOTORS_POSITION_PID_KD,
                TuningConstants.DRIVETRAIN_STEER_MOTORS_POSITION_PID_KF,
                DriveTrainMechanismNeo.DefaultPidSlotId);
            this.steerMotors[i].setPIDFSmartMotion(
                TuningConstants.DRIVETRAIN_STEER_MOTORS_MM_PID_KP,
                TuningConstants.DRIVETRAIN_STEER_MOTORS_MM_PID_KI,
                TuningConstants.DRIVETRAIN_STEER_MOTORS_MM_PID_KD,
                TuningConstants.DRIVETRAIN_STEER_MOTORS_MM_PID_KF,
                0,
                TuningConstants.DRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC,
                TuningConstants.DRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL,
                DriveTrainMechanismNeo.MMPidSlotId);
            this.steerMotors[i].setControlMode(SparkMaxControlMode.Position);

            if (TuningConstants.DRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC)
            {
                this.steerMotors[i].setSelectedSlot(DriveTrainMechanismNeo.MMPidSlotId);
            }
            else
            {
                this.steerMotors[i].setSelectedSlot(DriveTrainMechanismNeo.DefaultPidSlotId);
            }
        }

        this.driveVelocities = new double[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        this.drivePositions = new double[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        this.steerVelocities = new double[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        this.steerPositions = new double[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        this.steerAngles = new double[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        this.encoderAngles = new double[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];

        this.isDirectionSwapped = new boolean[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        this.driveSlotIds = new int[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];

        double prevYaw = this.robotYaw;
        double prevTime = this.time;
        this.robotYaw = this.imuManager.getYaw();
        this.time = this.timer.get();

        this.deltaT = this.time - prevTime;
        if (this.deltaT <= 0.0)
        {
            this.deltaT = 0.001;
        }

        this.omegaPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_OMEGA_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_OMEGA_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_OMEGA_MAX_OUTPUT,
            this.timer);

        this.pathOmegaPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_PATH_OMEGA_MAX_OUTPUT,
            this.timer);

        this.pathXOffsetPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_PATH_X_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_PATH_X_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_PATH_X_MAX_OUTPUT,
            this.timer);

        this.pathYOffsetPID = new PIDHandler(
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KP,
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KI,
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KD,
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KF,
            TuningConstants.DRIVETRAIN_PATH_Y_POSITION_PID_KS,
            TuningConstants.DRIVETRAIN_PATH_Y_MIN_OUTPUT,
            TuningConstants.DRIVETRAIN_PATH_Y_MAX_OUTPUT,
            this.timer);

        this.result = new Setpoint[DriveTrainMechanismNeo.NUM_DRIVE_MODULES];
        for (int i = 0; i < DriveTrainMechanismNeo.NUM_DRIVE_MODULES; i++)
        {
            this.result[i] = new Setpoint();
        }

        if (TuningConstants.DRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING)
        {
            this.xVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.DRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.DRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE,
                0.0);

            this.yVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.DRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.DRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE,
                0.0);
        }
        else
        {
            this.xVelocityLimiter = null;
            this.yVelocityLimiter = null;
        }

        if (TuningConstants.DRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING)
        {
            this.angularVelocityLimiter = new SlewRateLimiter(
                this.timer,
                TuningConstants.DRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE,
                TuningConstants.DRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE,
                0.0);
        }
        else
        {
            this.angularVelocityLimiter = null;
        }

        this.time = 0.0;
        this.angle = 0.0;
        this.xPosition = 0.0;
        this.yPosition = 0.0;

        this.firstRun = TuningConstants.DRIVETRAIN_RESET_ON_ROBOT_START;
        this.fieldOriented = TuningConstants.DRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START;
        this.maintainOrientation = TuningConstants.DRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START;
    }

    @Override
    public void readSensors()
    {
        for (int i = 0; i < DriveTrainMechanismNeo.NUM_DRIVE_MODULES; i++)
        {
            this.driveVelocities[i] = this.driveMotors[i].getVelocity();
            this.drivePositions[i] = this.driveMotors[i].getPosition();
            this.steerVelocities[i] = this.steerMotors[i].getVelocity();
            this.steerPositions[i] = this.steerMotors[i].getPosition();
            this.steerAngles[i] = Helpers.updateAngleRange(this.steerPositions[i] * HardwareConstants.DRIVETRAIN_STEER_TICK_DISTANCE);
            this.encoderAngles[i] = this.absoluteEncoders[i].getAbsolutePosition();

            this.logger.logNumber(DriveTrainMechanismNeo.DRIVE_VELOCITY_LOGGING_KEYS[i], this.driveVelocities[i]);
            this.logger.logNumber(DriveTrainMechanismNeo.DRIVE_POSITION_LOGGING_KEYS[i], this.drivePositions[i]);
            this.logger.logNumber(DriveTrainMechanismNeo.STEER_VELOCITY_LOGGING_KEYS[i], this.steerVelocities[i]);
            this.logger.logNumber(DriveTrainMechanismNeo.STEER_POSITION_LOGGING_KEYS[i], this.steerPositions[i]);
            this.logger.logNumber(DriveTrainMechanismNeo.STEER_ANGLE_LOGGING_KEYS[i], this.steerAngles[i]);
            this.logger.logNumber(DriveTrainMechanismNeo.ENCODER_ANGLE_LOGGING_KEYS[i], this.encoderAngles[i]);
        }
        
        double prevYaw = this.robotYaw;
        double prevTime = this.time;
        this.robotYaw = this.imuManager.getYaw();
        this.time = this.timer.get();

        //Why keep this postive?
        this.deltaT = this.time - prevTime;
        if (this.deltaT <= 0.0)
        {
            // keep this positive...
            this.deltaT = 0.001;
        }

        if (TuningConstants.DRIVETRAIN_USE_ODOMETRY)
        {
            double deltaImuYaw = (this.robotYaw - prevYaw) / this.deltaT;
            this.calculateOdometry(deltaImuYaw);
            this.logger.logNumber(LoggingKey.DriveTrainXPosition, this.xPosition);
            this.logger.logNumber(LoggingKey.DriveTrainYPosition, this.yPosition);
            this.logger.logNumber(LoggingKey.DriveTrainAngle, this.angle);
        }
    }

    @Override
    public void update() {
        
        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableFieldOrientation))
        {
            this.fieldOriented = true;
            this.desiredYaw = this.robotYaw;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainDisableFieldOrientation) ||
            !this.imuManager.getIsConnected())
        {
            this.fieldOriented = false;
        }

        boolean useFieldOriented = this.fieldOriented && !this.driver.getDigital(DigitalOperation.DriveTrainUseRobotOrientation);

        if (this.driver.getDigital(DigitalOperation.DriveTrainEnableMaintainDirectionMode))
        {
            this.maintainOrientation = true;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainDisableMaintainDirectionMode) ||
            !this.imuManager.getIsConnected())
        {
            this.maintainOrientation = false;
        }

        this.logger.logBoolean(LoggingKey.DriveTrainFieldOriented, useFieldOriented);
        this.logger.logBoolean(LoggingKey.DriveTrainMaintainOrientation, this.maintainOrientation);

        if (this.driver.getDigital(DigitalOperation.PositionResetFieldOrientation))
        {
            this.robotYaw = this.imuManager.getYaw();
            this.desiredYaw = this.robotYaw;
            this.angle = 0.0;
        }

        if (this.driver.getDigital(DigitalOperation.DriveTrainResetXYPosition))
        {
            this.xPosition = this.driver.getAnalog(AnalogOperation.DriveTrainStartingXPosition);
            this.yPosition = this.driver.getAnalog(AnalogOperation.DriveTrainStartingYPosition);
        }

        double startingAngle = this.driver.getAnalog(AnalogOperation.PositionStartingAngle);
        if (startingAngle != TuningConstants.ZERO)
        {
            this.angle = startingAngle;
        }

        if (this.firstRun || this.driver.getDigital(DigitalOperation.DriveTrainReset))
        {
            for (int i = 0; i < DriveTrainMechanismNeo.NUM_DRIVE_MODULES; i++)
            {
                this.driveMotors[i].setPosition(0);
                double angleDifference = (this.encoderAngles[i] - this.drivetrainSteerMotorAbsoluteOffsets[i]);
                double tickDifference = angleDifference * HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE;
                this.steerMotors[i].setPosition((int)tickDifference);

                this.drivePositions[i] = 0;
                this.steerPositions[i] = (int)tickDifference;
                this.steerAngles[i] = angleDifference % 360.0;
            }

            this.firstRun = false;
        }

        this.calculateSetpoints(useFieldOriented);
        for (int i = 0; i < NUM_DRIVE_MODULES; i++)
        {
            Setpoint current = this.result[i];
            Double steerSetpoint = current.angle;
            Double driveVelocitySetpoint = current.driveVelocity;
            Double drivePositionSetpoint = current.drivePosition;

            SparkMaxControlMode driveControlMode = SparkMaxControlMode.Disabled; // Disabled doesn't exist for SparkMax
            int driveDesiredPidSlotId = DefaultPidSlotId;
            double driveSetpoint = 0.0;
            if (driveVelocitySetpoint != null)
            {
                driveSetpoint = driveVelocitySetpoint;
                driveControlMode = SparkMaxControlMode.Velocity;
                driveDesiredPidSlotId = DefaultPidSlotId;
            }
            else if (drivePositionSetpoint != null)
            {
                driveSetpoint = drivePositionSetpoint;
                driveControlMode = SparkMaxControlMode.Position;
                driveDesiredPidSlotId = MMPidSlotId;
            }

            this.logger.logNumber(DRIVE_GOAL_LOGGING_KEYS[i], driveSetpoint);
            this.driveMotors[i].setControlMode(driveControlMode);
            if (driveControlMode != SparkMaxControlMode.Disabled)
            {
                this.driveMotors[i].set(driveSetpoint);

                if (driveDesiredPidSlotId != this.driveSlotIds[i])
                {
                    this.driveMotors[i].setSelectedSlot(driveDesiredPidSlotId);
                    this.driveSlotIds[i] = driveDesiredPidSlotId;
                }
            }
            else
            {
                this.driveMotors[i].stop();
            }

            if (steerSetpoint != null)
            {
                this.logger.logNumber(STEER_GOAL_LOGGING_KEYS[i], steerSetpoint);
                this.steerMotors[i].set(steerSetpoint);
            }
        }
    }

    @Override
    public void stop()
    {
        this.omegaPID.reset();
        this.pathOmegaPID.reset();
        this.pathXOffsetPID.reset();
        this.pathYOffsetPID.reset();
        for (int i = 0; i < DriveTrainMechanismNeo.NUM_DRIVE_MODULES; i++)
        {
            this.driveMotors[i].stop();
            this.steerMotors[i].stop();
        }

        if (this.xVelocityLimiter != null)
        {
            this.xVelocityLimiter.reset();
        }

        if (this.yVelocityLimiter != null)
        {
            this.yVelocityLimiter.reset();
        }

        if (this.angularVelocityLimiter != null)
        {
            this.angularVelocityLimiter.reset();
        }

        this.xPosition = 0.0;
        this.yPosition = 0.0;
    }

    public double[] getModuleTurnInPlaceAngles()
    {
        return new double[]
            {
                Helpers.atan2d(-this.moduleOffsetY[0], -this.moduleOffsetX[0]),
                Helpers.atan2d(-this.moduleOffsetY[1], -this.moduleOffsetX[1]),
                Helpers.atan2d(-this.moduleOffsetY[2], -this.moduleOffsetX[2]),
                Helpers.atan2d(-this.moduleOffsetY[3], -this.moduleOffsetX[3]),
            };
    }

    public double[] getDriveMotorPositions()
    {
        return new double[]
            {
                this.drivePositions[0],
                this.drivePositions[1],
                this.drivePositions[2],
                this.drivePositions[3],
            };
    }

    public Pose2d getPose()
    {
        return new Pose2d(this.xPosition, this.yPosition, this.robotYaw);
    }

    private void calculateSetpoints(boolean useFieldOriented)
    {
        boolean maintainPositionMode = this.driver.getDigital(DigitalOperation.DriveTrainMaintainPositionMode);
        if (maintainPositionMode || this.driver.getDigital(DigitalOperation.DriveTrainSteerMode))
        {
            for (int i = 0; i < NUM_DRIVE_MODULES; i++)
            {
                this.result[i].driveVelocity = null;
                if (maintainPositionMode)
                {
                    this.result[i].drivePosition = this.driver.getAnalog(DriveTrainMechanismNeo.DRIVE_SETPOINT_OPERATIONS[i]);
                }
                else
                {
                    this.result[i].drivePosition = null;
                }

                double moduleSteerPositionGoal = this.driver.getAnalog(DriveTrainMechanismNeo.STEER_SETPOINT_OPERATIONS[i]);
                double currentAngle = this.steerPositions[i] * HardwareConstants.DRIVETRAIN_STEER_TICK_DISTANCE;
                AnglePair anglePair = AnglePair.getClosestAngle(moduleSteerPositionGoal, currentAngle, true);
                moduleSteerPositionGoal = anglePair.getAngle() * TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;
                this.isDirectionSwapped[i] = anglePair.getSwapDirection();

                this.result[i].angle = moduleSteerPositionGoal;
            }

            return;
        }
    }

    private void calculateOdometry(double deltaImuYaw)
    {
        // double imuOmega = deltaImuYaw / this.deltaT; // in degrees
        double rightRobotVelocity;
        double forwardRobotVelocity;

        // calculate our right and forward velocities using an average of our various velocities and the angle.
        double rightRobotVelocity1 = -Helpers.sind(this.steerAngles[0]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[0];
        double rightRobotVelocity2 = -Helpers.sind(this.steerAngles[1]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[1];
        double rightRobotVelocity3 = -Helpers.sind(this.steerAngles[2]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[2];
        double rightRobotVelocity4 = -Helpers.sind(this.steerAngles[3]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[3];

        double forwardRobotVelocity1 = Helpers.cosd(this.steerAngles[0]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[0];
        double forwardRobotVelocity2 = Helpers.cosd(this.steerAngles[1]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[1];
        double forwardRobotVelocity3 = Helpers.cosd(this.steerAngles[2]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[2];
        double forwardRobotVelocity4 = Helpers.cosd(this.steerAngles[3]) * HardwareConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND * this.driveVelocities[3];

        // rightRobotVelocity = (rightRobotVelocity1 + rightRobotVelocity2 + rightRobotVelocity3 + rightRobotVelocity4) / 4.0;
        // forwardRobotVelocity = (forwardRobotVelocity1 + forwardRobotVelocity2 + forwardRobotVelocity3 + forwardRobotVelocity4) / 4.0;

        double a = 0.5 * (-rightRobotVelocity3 - rightRobotVelocity4);
        double b = 0.5 * (-rightRobotVelocity1 - rightRobotVelocity2);
        double c = 0.5 * (forwardRobotVelocity1 + forwardRobotVelocity4);
        double d = 0.5 * (forwardRobotVelocity2 + forwardRobotVelocity3);

        double omegaRadians1 = (b - a) / HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE;
        double omegaRadians2 = (c - d) / HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE;
        double omegaRadians = (omegaRadians1 + omegaRadians2) / 2.0;

        double rightRobotVelocityA = omegaRadians * HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE + a;
        double rightRobotVelocityB = -omegaRadians * HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE + b;
        rightRobotVelocity = -(rightRobotVelocityA + rightRobotVelocityB) / 2.0;

        double forwardRobotVelocityA = omegaRadians * HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + c;
        double forwardRobotVelocityB = -omegaRadians * HardwareConstants.DRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE + d;
        forwardRobotVelocity = (forwardRobotVelocityA + forwardRobotVelocityB) / 2.0;

        this.angle += omegaRadians * Helpers.RADIANS_TO_DEGREES * this.deltaT;

        double rightFieldVelocity = rightRobotVelocity * Helpers.cosd(this.robotYaw) - forwardRobotVelocity * Helpers.sind(this.robotYaw);
        double forwardFieldVelocity = rightRobotVelocity * Helpers.sind(this.robotYaw) + forwardRobotVelocity * Helpers.cosd(this.robotYaw);
        this.xPosition += forwardFieldVelocity * this.deltaT;
        this.yPosition -= rightFieldVelocity * this.deltaT;
    }
    
    /**
     * Basic structure to hold an angle/drive pair
     */
    private class Setpoint
    {
        public Double angle;
        public Double driveVelocity;
        public Double drivePosition;

        public Setpoint()
        {
        }
    }

    public double getPositionX()
    {
        return this.xPosition;
    }

    public double getPositionY()
    {
        return this.yPosition;
    }
}