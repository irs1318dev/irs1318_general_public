package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakeSpinTask extends TimedTask
{
    private final boolean out;

    public IntakeSpinTask(double duration, boolean out)
    {
        super(duration);

        this.out = out;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(DigitalOperation.ShooterLowerKicker, true);
        this.setDigitalOperationState(DigitalOperation.IntakeRotatingIn, !this.out);
        this.setDigitalOperationState(DigitalOperation.IntakeRotatingOut, this.out);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ShooterLowerKicker, true);
        this.setDigitalOperationState(DigitalOperation.IntakeRotatingIn, !this.out);
        this.setDigitalOperationState(DigitalOperation.IntakeRotatingOut, this.out);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.ShooterLowerKicker, false);
        this.setDigitalOperationState(DigitalOperation.IntakeRotatingIn, false);
        this.setDigitalOperationState(DigitalOperation.IntakeRotatingOut, false);
    }
}
