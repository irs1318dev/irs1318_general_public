package frc.robot.mechanisms;

import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;

public class OneMotorSparkMechanism implements IMechanism
{
    private static final String LogName = "om_spark";
    private static final int slotId = 1;

    private final IDashboardLogger logger;
    private final ISparkMax motor;

    private Driver driver;

    private double velocity;
    //private double error;
    private double ticks;
    public boolean reverseLimitSwtichStatus;
    public boolean forwardLimitSwitchStatus;

    @Inject
    public OneMotorSparkMechanism(
        IDashboardLogger logger,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.motor = provider.getSparkMax(ElectronicsConstants.ONEMOTOR_MASTER_MOTOR_CHANNEL, SparkMaxMotorType.Brushless);

        this.motor.setNeutralMode(MotorNeutralMode.Brake);
        this.motor.setInvertOutput(TuningConstants.ONEMOTOR_INVERT_OUTPUT);

        ISparkMax follower = provider.getSparkMax(ElectronicsConstants.ONEMOTOR_FOLLOWER_MOTOR_CHANNEL, SparkMaxMotorType.Brushless);
        follower.setNeutralMode(MotorNeutralMode.Brake);
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
                    OneMotorSparkMechanism.slotId);
            }

            this.motor.set(OneMotorSparkMechanism.slotId);
        }
        else
        {
            this.motor.setControlMode(SparkMaxControlMode.PercentOutput);
        }

        this.velocity = 0.0;
        //this.error = 0.0;
        this.ticks = 0;
        this.reverseLimitSwtichStatus = false;
        this.forwardLimitSwitchStatus = false;
    }

    @Override
    public void readSensors()
    {
        this.velocity = this.motor.getVelocity();
        //this.error = this.motor.get();
        this.ticks = this.motor.getPosition();

        if (TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED || TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED)
        {
            this.forwardLimitSwitchStatus = this.motor.getForwardLimitSwitchStatus();
            this.reverseLimitSwtichStatus = this.motor.getReverseLimitSwitchStatus();
        }

        this.logger.logNumber("om", "velocity", this.velocity);
        //this.logger.logNumber("om", "error", this.error);
        this.logger.logNumber("om", "position", this.ticks);
        this.logger.logBoolean("om", "reverseLimitSwtich", this.reverseLimitSwtichStatus);
        this.logger.logBoolean("om", "forwardLimitSwtich", this.forwardLimitSwitchStatus);
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

        this.logger.logNumber("om", "setpoint", setpoint);
        this.motor.set(setpoint);

        /*if (TuningConstants.ONEMOTOR_USE_PID)
        {
            double errorPercentage = 0.0;
            if (setpoint != 0.0)
            {
                errorPercentage = 100.0 * (this.error / maxSetpointValue);
            }

            this.logger.logNumber("om", "error%", errorPercentage);
        }*/
    }

    @Override
    public void stop()
    {
        this.motor.reset();
        this.motor.stop();
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }
}
