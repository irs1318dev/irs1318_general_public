package org.usfirst.frc.team1318.robot.onemotor;

import org.usfirst.frc.team1318.robot.common.DashboardLogger;

import com.ctre.CANTalon;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.google.inject.name.Named;

@Singleton
public class OneMotorComponent
{
    private static final String LogName = "om";

    private final CANTalon motor;

    @Inject
    public OneMotorComponent(@Named("ONEMOTOR_MOTOR") CANTalon motor)
    {
        this.motor = motor;
    }

    public void setPower(double power)
    {
        DashboardLogger.logNumber(OneMotorComponent.LogName, "Current Motor Power", power);
        this.motor.set(power);
    }
}
