package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.*;

import com.google.inject.Inject;

public class OneMotorSRXMechanism implements IMechanism
{
    private static final int slotId = 0;

    private final IDriver driver;
    private final ILogger logger;
    private final ITalonSRX motor;

    private double velocity;
    private double error;
    private double position;
    public boolean reverseLimitSwtichStatus;
    public boolean forwardLimitSwitchStatus;

    @Inject
    public OneMotorSRXMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;
        this.motor = provider.getTalonSRX(ElectronicsConstants.ONEMOTOR_PRIMARY_MOTOR_CHANNEL);

        this.motor.setSensorType(TalonXFeedbackDevice.QuadEncoder);
        this.motor.setNeutralMode(MotorNeutralMode.Brake);
        this.motor.setInvertOutput(TuningConstants.ONEMOTOR_INVERT_OUTPUT);
        this.motor.setInvertSensor(TuningConstants.ONEMOTOR_INVERT_SENSOR);
        this.motor.setForwardLimitSwitch(
            TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.motor.setReverseLimitSwitch(
            TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);

        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            if (TuningConstants.ONEMOTOR_PID_POSITIONAL)
            {
                if (TuningConstants.ONEMOTOR_PID_POSITIONAL_MM)
                {
                    this.motor.setControlMode(TalonXControlMode.MotionMagicPosition);

                    this.motor.setMotionMagicPIDF(
                        TuningConstants.ONEMOTOR_PID_KP,
                        TuningConstants.ONEMOTOR_PID_KI,
                        TuningConstants.ONEMOTOR_PID_KD,
                        TuningConstants.ONEMOTOR_PID_KF,
                        TuningConstants.ONEMOTOR_PID_MM_CRUISE_VELOC,
                        TuningConstants.ONEMOTOR_PID_MM_ACCEL,
                        OneMotorSRXMechanism.slotId);
                }
                else
                {
                    this.motor.setControlMode(TalonXControlMode.Position);

                    this.motor.setPIDF(
                        TuningConstants.ONEMOTOR_PID_KP,
                        TuningConstants.ONEMOTOR_PID_KI,
                        TuningConstants.ONEMOTOR_PID_KD,
                        TuningConstants.ONEMOTOR_PID_KF,
                        OneMotorSRXMechanism.slotId);
                }
            }
            else
            {
                this.motor.setControlMode(TalonXControlMode.Velocity);

                this.motor.setPIDF(
                    TuningConstants.ONEMOTOR_PID_KP,
                    TuningConstants.ONEMOTOR_PID_KI,
                    TuningConstants.ONEMOTOR_PID_KD,
                    TuningConstants.ONEMOTOR_PID_KF,
                    OneMotorSRXMechanism.slotId);
            }

            this.motor.setSelectedSlot(OneMotorSRXMechanism.slotId);
        }
        else
        {
            this.motor.setControlMode(TalonXControlMode.PercentOutput);
        }

        this.velocity = 0.0;
        this.error = 0.0;
        this.position = 0.0;
        this.reverseLimitSwtichStatus = false;
        this.forwardLimitSwitchStatus = false;
    }

    @Override
    public void readSensors()
    {
        this.velocity = this.motor.getVelocity();
        this.error = this.motor.getError();
        this.position = this.motor.getPosition();

        if (TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED || TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED)
        {
            TalonXLimitSwitchStatus limitSwitchStatus = this.motor.getLimitSwitchStatus();
            this.reverseLimitSwtichStatus = limitSwitchStatus.isReverseClosed;
            this.forwardLimitSwitchStatus = limitSwitchStatus.isForwardClosed;
        }

        this.logger.logNumber(LoggingKey.OneMotorSRXVelocity, this.velocity);
        this.logger.logNumber(LoggingKey.OneMotorSRXError, this.error);
        this.logger.logNumber(LoggingKey.OneMotorSRXPosition, this.position);
        this.logger.logBoolean(LoggingKey.OneMotorSRXReverseLimit, this.reverseLimitSwtichStatus);
        this.logger.logBoolean(LoggingKey.OneMotorSRXForwardLimit, this.forwardLimitSwitchStatus);
    }

    @Override
    public void update()
    {
        double setpoint = this.driver.getAnalog(AnalogOperation.OneMotorPower);

        double maxSetpointValue = 1.0;
        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            if (TuningConstants.ONEMOTOR_PID_POSITIONAL)
            {
                maxSetpointValue = TuningConstants.ONEMOTOR_PID_MAX_POSITION;
            }
            else
            {
                maxSetpointValue = TuningConstants.ONEMOTOR_PID_MAX_VELOCITY;
            }
        }

        setpoint *= maxSetpointValue;

        this.logger.logNumber(LoggingKey.OneMotorSRXSetpoint, setpoint);
        this.motor.set(setpoint);

        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            double errorPercentage = 0.0;
            if (setpoint != 0.0)
            {
                errorPercentage = 100.0 * (this.error / maxSetpointValue);
            }

            this.logger.logNumber(LoggingKey.OneMotorSRXErrorPercent, errorPercentage);
        }
    }

    @Override
    public void stop()
    {
        this.motor.reset();
        this.motor.stop();
    }
}