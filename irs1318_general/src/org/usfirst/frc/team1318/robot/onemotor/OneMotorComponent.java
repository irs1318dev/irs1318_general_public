package org.usfirst.frc.team1318.robot.onemotor;

import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.CANTalonControlMode;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ICANTalon;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.google.inject.name.Named;

@Singleton
public class OneMotorComponent
{
    private static final String LogName = "om";

    private final IDashboardLogger logger;
    private final ICANTalon motor;

    @Inject
    public OneMotorComponent(
        IDashboardLogger logger,
        @Named("ONEMOTOR_MOTOR") ICANTalon motor
    )
    {
        this.logger = logger;
        this.motor = motor;
    }

    public void setPower(double power)
    {
        this.logger.logNumber(OneMotorComponent.LogName, "setting", power);
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

    public double getSpeed()
    {
        return this.motor.getSpeed();
    }

    public double getError()
    {
        return this.motor.getError();
    }
}
