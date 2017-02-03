package org.usfirst.frc.team1318.robot.onemotor;

import org.usfirst.frc.team1318.robot.common.DashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ICANTalon;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.google.inject.name.Named;

@Singleton
public class OneMotorComponent
{
    private static final String LogName = "om";

    private final ICANTalon motor;

    @Inject
    public OneMotorComponent(@Named("ONEMOTOR_MOTOR") ICANTalon motor)
    {
        this.motor = motor;
    }

    public void setPower(double power)
    {
        DashboardLogger.logNumber(OneMotorComponent.LogName, "setting", power);
        this.motor.set(power);
    }
}
