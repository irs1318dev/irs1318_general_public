package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ShooterKickerTask extends TimedTask
{
    private final boolean lower;

    public ShooterKickerTask(double duration, boolean lower)
    {
        super(duration);

        this.lower = lower;
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ShooterLowerKicker, this.lower);
    }

    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(DigitalOperation.ShooterLowerKicker, this.lower);
    }
}
