package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class StingerTask extends TimedTask
{
    private final boolean out;

    public StingerTask(double duration, boolean out)
    {
        super(duration);

        this.out = out;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.setDigitalOperationState(DigitalOperation.StingerIn, !this.out);
        this.setDigitalOperationState(DigitalOperation.StingerOut, this.out);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.StingerIn, !this.out);
        this.setDigitalOperationState(DigitalOperation.StingerOut, this.out);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.StingerIn, false);
        this.setDigitalOperationState(DigitalOperation.StingerOut, false);
    }
}
