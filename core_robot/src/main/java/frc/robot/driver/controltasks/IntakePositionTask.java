package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class IntakePositionTask extends TimedTask
{
    private final boolean out;

    public IntakePositionTask(double duration, boolean out)
    {
        super(duration);

        this.out = out;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(DigitalOperation.IntakeExtend, this.out);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, !this.out);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.IntakeExtend, this.out);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, !this.out);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.IntakeExtend, false);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, false);
    }
}
