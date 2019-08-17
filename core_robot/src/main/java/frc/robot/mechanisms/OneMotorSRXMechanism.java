package frc.robot.mechanisms;

import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;

public class OneMotorSRXMechanism implements IMechanism
{
    private static final String LogName = "om_srx";
    private static final int slotId = 0;

    private final IDashboardLogger logger;
    private final ITalonSRX motor;

    private Driver driver;

    private double velocity;
    private double error;
    private int ticks;
    public boolean reverseLimitSwtichStatus;
    public boolean forwardLimitSwitchStatus;

    @Inject
    public OneMotorSRXMechanism(
        IDashboardLogger logger,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.motor = provider.getTalonSRX(ElectronicsConstants.ONEMOTOR_MASTER_MOTOR_CHANNEL);

        this.motor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
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
                    this.motor.setControlMode(TalonSRXControlMode.MotionMagicPosition);

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
                    this.motor.setControlMode(TalonSRXControlMode.Position);

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
                this.motor.setControlMode(TalonSRXControlMode.Velocity);

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
            this.motor.setControlMode(TalonSRXControlMode.PercentOutput);
        }

        this.velocity = 0.0;
        this.error = 0.0;
        this.ticks = 0;
        this.reverseLimitSwtichStatus = false;
        this.forwardLimitSwitchStatus = false;
    }

    @Override
    public void readSensors()
    {
        this.velocity = this.motor.getVelocity();
        this.error = this.motor.getError();
        this.ticks = this.motor.getPosition();

        if (TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED || TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED)
        {
            TalonSRXLimitSwitchStatus limitSwitchStatus = this.motor.getLimitSwitchStatus();
            this.reverseLimitSwtichStatus = limitSwitchStatus.isReverseClosed;
            this.forwardLimitSwitchStatus = limitSwitchStatus.isForwardClosed;
        }

        this.logger.logNumber(OneMotorSRXMechanism.LogName, "velocity", this.velocity);
        this.logger.logNumber(OneMotorSRXMechanism.LogName, "error", this.error);
        this.logger.logNumber(OneMotorSRXMechanism.LogName, "position", this.ticks);
        this.logger.logBoolean(OneMotorSRXMechanism.LogName, "reverseLimitSwtich", this.reverseLimitSwtichStatus);
        this.logger.logBoolean(OneMotorSRXMechanism.LogName, "forwardLimitSwtich", this.forwardLimitSwitchStatus);
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

        this.logger.logNumber(OneMotorSRXMechanism.LogName, "setpoint", setpoint);
        this.motor.set(setpoint);

        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            double errorPercentage = 0.0;
            if (setpoint != 0.0)
            {
                errorPercentage = 100.0 * (this.error / maxSetpointValue);
            }

            this.logger.logNumber(OneMotorSRXMechanism.LogName, "error%", errorPercentage);
        }
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
