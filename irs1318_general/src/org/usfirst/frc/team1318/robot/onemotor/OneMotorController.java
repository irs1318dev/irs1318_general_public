package org.usfirst.frc.team1318.robot.onemotor;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.DashboardLogger;
import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.driver.Driver;
import org.usfirst.frc.team1318.robot.driver.Operation;

import com.google.inject.Inject;

public class OneMotorController implements IController
{
    private Driver driver;

    private OneMotorComponent component;

    @Inject
    public OneMotorController(OneMotorComponent component)
    {
        this.component = component;
    }

    @Override
    public void update()
    {
        double power = this.driver.getAnalog(Operation.OneMotorPower);

        if (TuningConstants.ONEMOTOR_USE_PID)
        {
            power *= TuningConstants.ONEMOTOR_PID_MAX_VELOCITY;
        }

        DashboardLogger.logNumber("om", "power/setpoint", power);

        // apply the power settings to the drivetrain component
        this.component.setPower(power);

        double ICANTalonVelocity = this.component.getEncVelocity();
        DashboardLogger.logNumber("om", "encoderVelocity", ICANTalonVelocity);
    }

    @Override
    public void stop()
    {
        this.component.setPower(0.0);
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }
}
