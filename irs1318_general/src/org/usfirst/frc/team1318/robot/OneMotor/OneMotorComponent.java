package org.usfirst.frc.team1318.robot.OneMotor;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.Common.DashboardLogger;

import com.ctre.CANTalon;

public class OneMotorComponent
{
    //    private final Encoder encoder;
    private final CANTalon motor;

    public OneMotorComponent()
    {
        this.motor = new CANTalon(ElectronicsConstants.ONEMOTOR_MOTOR_CHANNEL);

        //        this.encoder = new Encoder(
        //            ElectronicsConstants.ONEMOTOR_ENCODER_CHANNEL_A,
        //            ElectronicsConstants.ONEMOTOR_ENCODER_CHANNEL_B);
    }

    public void setPower(double power)
    {
        DashboardLogger.putDouble("Current Motor Power", power);
        this.motor.set(power);
    }

    //    public int getEncoderTicks()
    //    {
    //        int ticks = this.encoder.get();
    //        DashboardLogger.putInteger("Encoder Ticks", ticks);
    //        return ticks;
    //    }
}
