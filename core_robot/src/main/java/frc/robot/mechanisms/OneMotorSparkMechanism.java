package frc.robot.mechanisms;

import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.common.IMechanism;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.IDriver;

import com.google.inject.Inject;

public class OneMotorSparkMechanism implements IMechanism
{
    private static final int slotId = 1;

    private final IDriver driver;
    private final ILogger logger;
    private final ISparkMax motor;

    private double velocity;
    private double position;
    public boolean reverseLimitSwtichStatus;
    public boolean forwardLimitSwitchStatus;

    @Inject
    public OneMotorSparkMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;
        this.motor = provider.getSparkMax(ElectronicsConstants.ONEMOTOR_PRIMARY_MOTOR_CHANNEL, SparkMaxMotorType.Brushless);
        this.motor.setNeutralMode(MotorNeutralMode.Brake);
        this.motor.setInvertOutput(TuningConstants.ONEMOTOR_INVERT_OUTPUT);
        this.motor.reset();

        ISparkMax follower = provider.getSparkMax(ElectronicsConstants.ONEMOTOR_FOLLOWER_MOTOR_CHANNEL, SparkMaxMotorType.Brushless);
        follower.setNeutralMode(MotorNeutralMode.Brake);
        follower.reset();
        follower.follow(this.motor);

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
                    this.motor.setControlMode(SparkMaxControlMode.Position);

                    this.motor.setPIDFSmartMotion(
                        TuningConstants.ONEMOTOR_PID_KP,
                        TuningConstants.ONEMOTOR_PID_KI,
                        TuningConstants.ONEMOTOR_PID_KD,
                        TuningConstants.ONEMOTOR_PID_KF,
                        0,
                        TuningConstants.ONEMOTOR_PID_MM_CRUISE_VELOC,
                        TuningConstants.ONEMOTOR_PID_MM_ACCEL,
                        TuningConstants.ONEMOTOR_PID_MIN_OUTPUT,
                        TuningConstants.ONEMOTOR_PID_MAX_OUTPUT,
                        OneMotorSparkMechanism.slotId);
                }
                else
                {
                    this.motor.setControlMode(SparkMaxControlMode.Position);

                    this.motor.setPIDF(
                        TuningConstants.ONEMOTOR_PID_KP,
                        TuningConstants.ONEMOTOR_PID_KI,
                        TuningConstants.ONEMOTOR_PID_KD,
                        TuningConstants.ONEMOTOR_PID_KF,
                        TuningConstants.ONEMOTOR_PID_MIN_OUTPUT,
                        TuningConstants.ONEMOTOR_PID_MAX_OUTPUT,
                        OneMotorSparkMechanism.slotId);
                }
            }
            else
            {
                this.motor.setControlMode(SparkMaxControlMode.Velocity);

                this.motor.setPIDF(
                    TuningConstants.ONEMOTOR_PID_KP,
                    TuningConstants.ONEMOTOR_PID_KI,
                    TuningConstants.ONEMOTOR_PID_KD,
                    TuningConstants.ONEMOTOR_PID_KF,
                    TuningConstants.ONEMOTOR_PID_MIN_OUTPUT,
                    TuningConstants.ONEMOTOR_PID_MAX_OUTPUT,
                    OneMotorSparkMechanism.slotId);
            }
        }
        else
        {
            this.motor.setControlMode(SparkMaxControlMode.PercentOutput);
        }

        this.velocity = 0.0;
        this.position = 0.0;
        this.reverseLimitSwtichStatus = false;
        this.forwardLimitSwitchStatus = false;
    }

    @Override
    public void readSensors()
    {
        this.velocity = this.motor.getVelocity();
        this.position = this.motor.getPosition();

        if (TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED || TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED)
        {
            this.forwardLimitSwitchStatus = this.motor.getForwardLimitSwitchStatus();
            this.reverseLimitSwtichStatus = this.motor.getReverseLimitSwitchStatus();
        }

        this.logger.logNumber(LoggingKey.OneMotorSparkVelocity, this.velocity);
        this.logger.logNumber(LoggingKey.OneMotorSparkPosition, this.position);
        this.logger.logBoolean(LoggingKey.OneMotorSparkReverseLimit, this.reverseLimitSwtichStatus);
        this.logger.logBoolean(LoggingKey.OneMotorSparkForwardLimit, this.forwardLimitSwitchStatus);
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

        this.logger.logNumber(LoggingKey.OneMotorSparkSetpoint, setpoint);
        this.motor.set(setpoint);
    }

    @Override
    public void stop()
    {
        this.motor.reset();
        this.motor.stop();
    }
}
