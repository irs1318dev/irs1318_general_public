package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ShooterMechanism;

public class ShooterSpinDownTask extends TimedTask
{
    private double startingSpeedPercent;

    public ShooterSpinDownTask(double duration)
    {
        super(duration);
    }

    @Override
    public void begin()
    {
        super.begin();

        ShooterMechanism shooter = this.getInjector().getInstance(ShooterMechanism.class);
        this.startingSpeedPercent = shooter.getRate() / TuningConstants.SHOOTER_MAX_COUNTER_RATE;

        this.setDigitalOperationState(DigitalOperation.ShooterSpin, true);
        this.setDigitalOperationState(DigitalOperation.ShooterExtendHood, false);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ShooterSpin, true);
        this.setAnalogOperationState(AnalogOperation.ShooterSpeed, (1.0 - this.getRatioComplete()) * this.startingSpeedPercent);
        this.setDigitalOperationState(DigitalOperation.ShooterExtendHood, false);
    }

    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(DigitalOperation.ShooterSpin, false);
        this.setAnalogOperationState(AnalogOperation.ShooterSpeed, 0.0);
        this.setDigitalOperationState(DigitalOperation.ShooterExtendHood, false);
    }
}
