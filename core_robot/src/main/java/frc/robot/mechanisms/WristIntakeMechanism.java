package frc.robot.mechanisms;

 import frc.robot.*;
 import frc.lib.driver.*;
 import frc.lib.helpers.Helpers;
 import frc.lib.mechanisms.*;
 import frc.lib.robotprovider.*;
 import frc.robot.driver.*;
 import frc.lib.filters.FloatingAverageCalculator;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class WristIntakeMechanism implements IMechanism 
{
    private static final int DefaultPidSlotId = 0;
    private static final int SMPidSlotId = 1;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;
    private final PowerManager powerManager;

    private double prevTime;

    private final ISparkMax wristMotor;
    private final ISparkMax intakeMotor;

    private double wristMotorAngle;
    private double wristMotorVelocity;
    private double wristMotorDesiredAngle;

    private double wristPowerAverage;
    private double wristVelocityAverage;

    private FloatingAverageCalculator wristPowerAverageCalculator;
    private FloatingAverageCalculator wristVelocityAverageCalculator;

    private double wristSetpointChangedTime;
    private boolean wristStalled;    

    private double intakeMotorVelocity;

    private boolean inSimpleMode;

    @Inject
    public WristIntakeMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        PowerManager powerManager,
        ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.powerManager = powerManager;

        this.intakeMotor = provider.getSparkMax(ElectronicsConstants.INTAKE_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);
        this.intakeMotor.setRelativeEncoder();
        this.intakeMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        this.intakeMotor.setInvertOutput(TuningConstants.INTAKE_MOTOR_INVERT_OUTPUT);
        this.intakeMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.intakeMotor.burnFlash();

        this.wristMotor = provider.getSparkMax(ElectronicsConstants.WRIST_MOTOR_CAN_ID, SparkMaxMotorType.Brushed);
        this.wristMotor.setAbsoluteEncoder();
        this.wristMotor.setInvertSensor(TuningConstants.WRIST_MOTOR_INVERT_SENSOR);
        this.wristMotor.setPositionConversionFactor(HardwareConstants.WRIST_MOTOR_TICK_DISTANCE);
        this.wristMotor.setInvertOutput(TuningConstants.WRIST_MOTOR_INVERT_OUTPUT);
        this.wristMotor.setNeutralMode(MotorNeutralMode.Brake);

        this.wristMotor.setPIDF(
            TuningConstants.WRIST_MOTOR_POSITION_PID_KP, 
            TuningConstants.WRIST_MOTOR_POSITION_PID_KI,
            TuningConstants.WRIST_MOTOR_POSITION_PID_KD,
            TuningConstants.WRIST_MOTOR_POSITION_PID_KF,
            WristIntakeMechanism.DefaultPidSlotId);
        this.wristMotor.setPIDFSmartMotion(
            TuningConstants.WRIST_MOTOR_SM_PID_KP, 
            TuningConstants.WRIST_MOTOR_SM_PID_KI,
            TuningConstants.WRIST_MOTOR_SM_PID_KD,
            TuningConstants.WRIST_MOTOR_SM_PID_KF,
            TuningConstants.WRIST_MOTOR_SM_IZONE,
            TuningConstants.WRIST_MOTOR_SM_PID_CRUISE_VELOC,
            TuningConstants.WRIST_MOTOR_SM_PID_ACCEL,
            WristIntakeMechanism.SMPidSlotId);
        this.wristMotor.setPositionPIDWrappingSettings(
            TuningConstants.WRIST_MOTOR_POSITION_PID_WRAPPING_ENABLED,
            TuningConstants.WRIST_MOTOR_POSITION_PID_WRAPPING_MIN,
            TuningConstants.WRIST_MOTOR_POSITION_PID_WRAPPING_MAX);

        if (TuningConstants.WRIST_MOTOR_USE_SMART_MOTION)
        {
            this.wristMotor.setControlMode(SparkMaxControlMode.SmartMotionPosition);
            this.wristMotor.setSelectedSlot(WristIntakeMechanism.SMPidSlotId);
        }
        else
        {
            this.wristMotor.setControlMode(SparkMaxControlMode.Position);
            this.wristMotor.setSelectedSlot(WristIntakeMechanism.DefaultPidSlotId);
        }

        this.wristMotor.burnFlash();

        
        this.wristPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.WRIST_POWER_TRACKING_DURATION, TuningConstants.WRIST_POWER_SAMPLES_PER_SECOND);
        this.wristVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.WRIST_VELOCITY_TRACKING_DURATION, TuningConstants.WRIST_VELOCITY_SAMPLES_PER_SECOND);

        this.wristMotorDesiredAngle =
            Helpers.EnforceRange(this.wristMotor.getPosition(), HardwareConstants.WRIST_MIN_ANGLE, HardwareConstants.WRIST_MAX_ANGLE);
    }

    @Override
    public void readSensors()
    {
        this.wristMotorVelocity = this.wristMotor.getVelocity();
        this.wristMotorAngle = this.wristMotor.getPosition();

        this.intakeMotorVelocity = this.intakeMotor.getVelocity();

        double wristCurrent = this.powerManager.getCurrent(ElectronicsConstants.WRIST_PDH_CHANNEL);
        double batteryVoltage = this.powerManager.getBatteryVoltage();

        this.wristPowerAverage = this.wristPowerAverageCalculator.update(wristCurrent * batteryVoltage);
        this.wristVelocityAverage = this.wristVelocityAverageCalculator.update(Math.abs(this.wristMotorVelocity));


        
        this.logger.logNumber(LoggingKey.WristVelocityAverage, this.wristVelocityAverage);
        this.logger.logNumber(LoggingKey.WristPower, this.wristPowerAverage);
        this.logger.logNumber(LoggingKey.WristMotorVelocity, this.wristMotorVelocity);
        this.logger.logNumber(LoggingKey.WristMotorAngle, this.wristMotorAngle);
        this.logger.logNumber(LoggingKey.IntakeMotorVelocity, this.intakeMotorVelocity);
    }

    @Override
    public void update()
    {
        double currTime = this.timer.get();
        if (this.driver.getDigital(DigitalOperation.WristEnableSimpleMode))
        {
            this.inSimpleMode = true;
            this.wristMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        }
        else if (this.driver.getDigital(DigitalOperation.WristDisableSimpleMode))
        {
            this.inSimpleMode = false;
            this.wristMotorDesiredAngle = this.wristMotorAngle;
            this.wristMotor.setControlMode(SparkMaxControlMode.Position);

            this.wristSetpointChangedTime = currTime;
            this.wristStalled = false;
        }

        // --------------------------------- Intake Update -----------------------------------------------------

        // control intake rollers
        double wristIntakePower = TuningConstants.ZERO;
        if (this.driver.getDigital(DigitalOperation.IntakeIn))
        {
            wristIntakePower = TuningConstants.WRIST_INTAKE_IN_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeOut))
        {
            wristIntakePower = TuningConstants.WRIST_INTAKE_OUT_POWER;
        }

        this.intakeMotor.set(wristIntakePower);
        this.logger.logNumber(LoggingKey.intakeMotorPercentOutput, wristIntakePower);

        // -------------------------------------- Main Wrist ----------------------------------------------------

        double wristAngleAdjustment = this.driver.getAnalog(AnalogOperation.WristAngleAdjustment);

        double wristPower = 0.0;
        if (this.inSimpleMode)
        {
            // controlled by joystick - raw power
            wristPower = wristAngleAdjustment;

            this.wristSetpointChangedTime = currTime;
            this.wristStalled = false;
        }
        else
        {
            double newDesiredWristAngle = this.driver.getAnalog(AnalogOperation.WristSetAngle);
            double elapsedTime = currTime - this.prevTime;

            if (wristAngleAdjustment != 0.0)
            {
                // Controlled by joysticks - angle adjustment
                this.wristMotorDesiredAngle += wristAngleAdjustment * TuningConstants.WRIST_INPUT_TO_TICK_ADJUSTMENT * elapsedTime;

                this.wristSetpointChangedTime = currTime;
                this.wristStalled = false;
            }
            else if (newDesiredWristAngle != TuningConstants.MAGIC_NULL_VALUE)
            {
                // controlled by macro
                if (!Helpers.RoughEquals(this.wristMotorDesiredAngle, newDesiredWristAngle, 0.1))
                {
                    this.wristMotorDesiredAngle = newDesiredWristAngle;

                    this.wristSetpointChangedTime = currTime;
                    this.wristStalled = false;
                }
            }
        }

        if (TuningConstants.WRIST_STALL_PROTECTION_ENABLED)
        {
            if (currTime > this.wristSetpointChangedTime + TuningConstants.WRIST_VELOCITY_TRACKING_DURATION &&
                this.wristPowerAverage >= TuningConstants.WRIST_STALLED_POWER_THRESHOLD &&
                Math.abs(this.wristVelocityAverage) <= TuningConstants.WRIST_STALLED_VELOCITY_THRESHOLD)
            {
                this.wristStalled = true;
            }
        }

        if (!this.inSimpleMode)
        {
            if (this.wristStalled)
            {
                this.wristMotor.stop();
            }
            else 
            {
                this.wristMotorDesiredAngle = Helpers.EnforceRange(this.wristMotorDesiredAngle, HardwareConstants.WRIST_MIN_ANGLE, HardwareConstants.WRIST_MAX_ANGLE);

                this.wristMotor.set(this.wristMotorDesiredAngle);
            }
        }
        else 
        {
            this.wristMotor.set(wristPower);
        }

        this.logger.logBoolean(LoggingKey.WristMotorStalled, this.wristStalled);
        this.logger.logNumber(LoggingKey.WristMotorSetPower, wristPower);
        this.logger.logNumber(LoggingKey.WristMotorSetPosition, this.wristMotorDesiredAngle);

        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.wristMotor.stop();
        this.intakeMotor.stop();
    }

    public double getPosition()
    {
        return this.wristMotorAngle;
    }
}