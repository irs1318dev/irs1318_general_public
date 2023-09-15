package frc.robot.mechanisms;

 import frc.robot.*;
 import frc.lib.driver.*;
 import frc.lib.helpers.Helpers;
 import frc.lib.mechanisms.*;
 import frc.lib.robotprovider.*;
 import frc.robot.driver.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class WristIntakeMechanism implements IMechanism 
{
    private static final int DefaultPidSlotId = 0;
    private static final int SMPidSlotId = 1;

    private final IDriver driver;
    private final ILogger logger;

    private final ISparkMax wristMotor;
    private final ISparkMax intakeMotor;

    private double wristMotorAngle;
    private double wristMotorVelocity;
    private double wristMotorDesiredAngle;

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

        this.wristMotor = provider.getSparkMax(TuningConstants.WRIST_MOTOR_CAN_ID, SparkMaxMotorType.Brushed);
        this.intakeMotor = provider.getSparkMax(TuningConstants.INTAKE_MOTOR_CAN_ID, SparkMaxMotorType.Brushless);

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
        
        this.wristMotor.setControlMode(SparkMaxControlMode.Position);
        this.wristMotor.burnFlash();

        this.intakeMotor.setControlMode(SparkMaxControlMode.PercentOutput);
        this.intakeMotor.setInvertOutput(TuningConstants.INTAKE_MOTOR_INVERT_OUTPUT);
        this.intakeMotor.setNeutralMode(MotorNeutralMode.Brake);

        if (TuningConstants.REVDRIVETRAIN_STEER_MOTORS_USE_SMART_MOTION)
        {
            this.wristMotor.setSelectedSlot(WristIntakeMechanism.SMPidSlotId);
        }
        else
        {
            this.wristMotor.setSelectedSlot(WristIntakeMechanism.DefaultPidSlotId);
        }
    }


    @Override
    public void readSensors()
    {
        this.wristMotorVelocity = this.wristMotor.getVelocity();
        this.wristMotorAngle = this.wristMotor.getPosition();

        this.intakeMotorVelocity = this.intakeMotor.getVelocity();

        this.logger.logNumber(LoggingKey.WristMotorVelocity, this.wristMotorVelocity);
        this.logger.logNumber(LoggingKey.WristMotorAngle, this.wristMotorAngle);
        this.logger.logNumber(LoggingKey.IntakeMotorVelocity, this.intakeMotorVelocity);
    }

    @Override
    public void update()
    {
        if (this.driver.getDigital(DigitalOperation.WristEnableSimpleMode))
        {
            this.inSimpleMode = true;
        }
        else if (this.driver.getDigital(DigitalOperation.WristDisableSimpleMode))
        {
            this.inSimpleMode = false;
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
        }

        else
        {
            double newDesiredWristAngle = this.driver.getAnalog(AnalogOperation.WristSetAngle);

            if (wristAngleAdjustment != 0.0)
            {
                this.wristMotorDesiredAngle = this.wristMotorAngle;

                // Controlled by joysticks - angle adjustment
                this.wristMotorDesiredAngle += wristAngleAdjustment * TuningConstants.WRIST_INPUT_TO_TICK_ADJUSTMENT;
            }
            
            else if (newDesiredWristAngle != TuningConstants.MAGIC_NULL_VALUE)
            {
                // controlled by macro
                if (newDesiredWristAngle != TuningConstants.MAGIC_NULL_VALUE &&
                    (!Helpers.RoughEquals(this.wristMotorDesiredAngle, newDesiredWristAngle, 0.1)))
                {
                    this.wristMotorDesiredAngle = newDesiredWristAngle;
                }
            }
        }

        if (!this.inSimpleMode)
        {
            
            this.wristMotor.set(this.wristMotorDesiredAngle);
        }
        else
        {
            this.wristMotor.set(wristPower);
        }
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