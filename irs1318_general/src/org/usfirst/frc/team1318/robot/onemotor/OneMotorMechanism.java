package org.usfirst.frc.team1318.robot.onemotor;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.CANTalonControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.ICANTalon;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

import com.google.inject.Inject;

public class OneMotorMechanism implements IMechanism
{
    private static final String LogName = "om";

    private final IDashboardLogger logger;
    private final ICANTalon motor;

    private Driver driver;

    @Inject
    public OneMotorMechanism(
        IDashboardLogger logger,
        IWpilibProvider provider)
    {
        this.logger = logger;
        this.motor = provider.getCANTalon(ElectronicsConstants.ONEMOTOR_MASTER_MOTOR_CHANNEL);

        this.motor.enableBrakeMode(false);
        this.motor.reverseSensor(false);

        //        ICANTalon follower = provider.getCANTalon(ElectronicsConstants.ONEMOTOR_FOLLOWER_MOTOR_CHANNEL);
        //        follower.enableBrakeMode(false);
        //        follower.reverseOutput(true);
        //        follower.changeControlMode(CANTalonControlMode.Follower);
        //        follower.set(ElectronicsConstants.ONEMOTOR_MASTER_MOTOR_CHANNEL);

        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            this.motor.changeControlMode(CANTalonControlMode.Speed);
            this.motor.setPIDF(
                TuningConstants.ONEMOTOR_PID_KP,
                TuningConstants.ONEMOTOR_PID_KI,
                TuningConstants.ONEMOTOR_PID_KD,
                TuningConstants.ONEMOTOR_PID_KF);
        }
        else
        {
            this.motor.changeControlMode(CANTalonControlMode.PercentVbus);
        }
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
        this.setPower(power);

        double velocity = this.getSpeed();
        this.logger.logNumber("om", "speed", velocity);

        double error = this.getError();
        this.logger.logNumber("om", "error", error);

        double errorPercentage = 0.0;
        if (power != 0.0)
        {
            errorPercentage = 100.0 * (error / power);
        }

        this.logger.logNumber("om", "error%", errorPercentage);
    }

    @Override
    public void stop()
    {
        this.setPower(0.0);
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    public double getSpeed()
    {
        return this.motor.getSpeed();
    }

    public double getError()
    {
        return this.motor.getError();
    }

    private void setPower(double power)
    {
        this.logger.logNumber(OneMotorMechanism.LogName, "setting", power);
        if (power == 0.0)
        {
            this.motor.changeControlMode(CANTalonControlMode.PercentVbus);
        }
        else
        {
            this.motor.changeControlMode(CANTalonControlMode.Speed);
        }

        this.motor.set(power);
    }
}
