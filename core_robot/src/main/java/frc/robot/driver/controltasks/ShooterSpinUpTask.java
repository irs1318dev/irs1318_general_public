package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ShooterMechanism;

public class ShooterSpinUpTask extends TimedTask
{
    private final double shooterVelocity;
    private final Boolean extendHood;

    private ShooterMechanism shooter;

    public ShooterSpinUpTask(double duration, double shooterVelocity)
    {
        this(duration, shooterVelocity, null);
    }

    public ShooterSpinUpTask(double duration, double shooterVelocity, boolean extendHood)
    {
        this(duration, shooterVelocity, (Boolean)extendHood);
    }

    private ShooterSpinUpTask(double duration, double shooterVelocity, Boolean extendHood)
    {
        super(duration);

        this.shooterVelocity = shooterVelocity;
        this.extendHood = extendHood;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.shooter = this.getInjector().getInstance(ShooterMechanism.class);

        this.setDigitalOperationState(DigitalOperation.ShooterSpin, true);
        this.setAnalogOperationState(AnalogOperation.ShooterSpeed, this.shooterVelocity);
        if (this.extendHood != null)
        {
            this.setDigitalOperationState(DigitalOperation.ShooterExtendHood, (boolean)this.extendHood);
        }
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ShooterSpin, true);
        this.setAnalogOperationState(AnalogOperation.ShooterSpeed, this.shooterVelocity);
        if (this.extendHood != null)
        {
            this.setDigitalOperationState(DigitalOperation.ShooterExtendHood, (boolean)this.extendHood);
        }
    }

    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(DigitalOperation.ShooterSpin, false);
        this.setAnalogOperationState(AnalogOperation.ShooterSpeed, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        if (super.hasCompleted())
        {
            return true;
        }

        double speed = this.shooter.getRate() / TuningConstants.SHOOTER_MAX_COUNTER_RATE;
        return Helpers.WithinDelta(speed, this.shooterVelocity, TuningConstants.SHOOTER_DEVIANCE);
    }
}
