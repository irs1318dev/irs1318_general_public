package org.usfirst.frc.team1318.robot.onemotor;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXNeutralMode;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

import com.google.inject.Inject;

public class OneMotorMechanism implements IMechanism
{
    private static final String LogName = "om";
    private static final int slotId = 0;

    private final IDashboardLogger logger;
    private final ITalonSRX motor;

    private Driver driver;

    private double velocity;
    private double error;
    private int ticks;

    @Inject
    public OneMotorMechanism(
        IDashboardLogger logger,
        IWpilibProvider provider)
    {
        this.logger = logger;
        this.motor = provider.getTalonSRX(ElectronicsConstants.ONEMOTOR_MASTER_MOTOR_CHANNEL);

        this.motor.setNeutralMode(TalonSRXNeutralMode.Coast);
        this.motor.setInvertSensor(false);

        //        ITalonSRX follower = provider.getTalonSRX(ElectronicsConstants.ONEMOTOR_FOLLOWER_MOTOR_CHANNEL);
        //        follower.setNeutralMode(TalonSRXNeutralMode.Coast);
        //        follower.reverseOutput(true);
        //        follower.changeControlMode(TalonSRXControlMode.Follower);
        //        follower.set(ElectronicsConstants.ONEMOTOR_MASTER_MOTOR_CHANNEL);

        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            this.motor.setControlMode(TalonSRXControlMode.Velocity);
            this.motor.setPIDF(
                TuningConstants.ONEMOTOR_PID_KP,
                TuningConstants.ONEMOTOR_PID_KI,
                TuningConstants.ONEMOTOR_PID_KD,
                TuningConstants.ONEMOTOR_PID_KF,
                OneMotorMechanism.slotId);
        }
        else
        {
            this.motor.setControlMode(TalonSRXControlMode.PercentOutput);
        }
    }

    public double getSpeed()
    {
        return this.velocity;
    }

    public double getError()
    {
        return this.error;
    }

    @Override
    public void readSensors()
    {
        this.velocity = this.motor.getVelocity();
        this.error = this.getError();
        this.ticks = this.motor.getPosition();

        this.logger.logNumber("om", "velocity", this.velocity);
        this.logger.logNumber("om", "error", this.error);
        this.logger.logNumber("om", "position", this.ticks);
    }

    @Override
    public void update()
    {
        double power = this.driver.getAnalog(Operation.OneMotorPower);

        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            power *= TuningConstants.ONEMOTOR_PID_MAX_VELOCITY;
        }

        this.logger.logNumber("om", "power/setpoint", power);

        // apply the power settings to the motor
        this.logger.logNumber(OneMotorMechanism.LogName, "setting", power);
        this.motor.setControlMode(TalonSRXControlMode.Velocity);
        this.motor.set(power);

        double errorPercentage = 0.0;
        if (power != 0.0)
        {
            errorPercentage = 100.0 * (this.error / power);
        }

        this.logger.logNumber("om", "error%", errorPercentage);
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
