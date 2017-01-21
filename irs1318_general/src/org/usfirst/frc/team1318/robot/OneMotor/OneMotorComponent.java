package org.usfirst.frc.team1318.robot.OneMotor;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.Common.DashboardLogger;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;

public class OneMotorComponent
{
    private final Encoder encoder;
    private final Victor talon;

    public OneMotorComponent()
    {
        this.talon = new Victor(
            ElectronicsConstants.ONEMOTOR_TALON_CHANNEL);

        this.encoder = new Encoder(
            ElectronicsConstants.ONEMOTOR_ENCODER_CHANNEL_A,
            ElectronicsConstants.ONEMOTOR_ENCODER_CHANNEL_B);
    }

    public void setPower(double power)
    {
        DashboardLogger.putDouble("Current Motor Power", power);
        this.talon.set(power);
    }

    public int getEncoderTicks()
    {
        int ticks = this.encoder.get();
        DashboardLogger.putInteger("Encoder Ticks", ticks);
        return ticks;
    }

}
