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
public class DriveTrainMechanismNeo implements IMechanism {

    private static final int NUM_DRIVE_MODULES = 4;

    private static final int DefaultPidSlotId = 0;
    private static final int MMPidSlotId = 1;

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
    private final double[] driveErrors;
    private final double[] steerVelocities;
    private final double[] steerPositions;
    private final double[] steerAngles;
    private final double[] steerErrors;
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

    DriveTrainMechanismNeo(IDriver driver, LoggingManager logger, IRobotProvider provider, PigeonManager imuManager, PowerManager powerManager, ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.imuManager = imuManager;
        this.powerManager = powerManager;

        this.steerMotors = new ISparkMax[DriveTrainMechanism.NUM_DRIVE_MODULES];
        this.driveMotors = new ISparkMax[DriveTrainMechanism.NUM_DRIVE_MODULES];
        this.absoluteEncoders = new ICANCoder[DriveTrainMechanism.NUM_DRIVE_MODULES];

        this.moduleOffsetX =
        new double[]
        {
            -HardwareConstants.DRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE, //Module 1: front-right
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


        //Change Types
        TalonFXInvertType[] driveMotorInvert =
            new TalonFXInvertType[]
            {
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR1_INVERT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR2_INVERT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR3_INVERT,
                HardwareConstants.DRIVETRAIN_DRIVE_MOTOR4_INVERT
            };

        TalonFXInvertType[] steerMotorInvert =
            new TalonFXInvertType[]
            {
                HardwareConstants.DRIVETRAIN_STEER_MOTOR1_INVERT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR2_INVERT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR3_INVERT,
                HardwareConstants.DRIVETRAIN_STEER_MOTOR4_INVERT
            };



    }

    @Override
    public void readSensors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }
}

private class Setpoint()
{
    public Double angle;
    public Double driveVelocity;
    public Double drivePosition;

    public Setpoint()
    {
    }
}