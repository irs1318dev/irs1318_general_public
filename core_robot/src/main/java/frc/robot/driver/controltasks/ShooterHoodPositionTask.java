package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class ShooterHoodPositionTask extends CompositeOperationTask
{
    private static final DigitalOperation[] HOOD_POSITIONS =
        new DigitalOperation[]
        {
            DigitalOperation.PowerCellHoodPointBlank,
            DigitalOperation.PowerCellHoodShort,
            DigitalOperation.PowerCellHoodMedium,
            DigitalOperation.PowerCellHoodLong,
        };

    /**
     * Initializes a new FlywheelFixedSpinTask
     * @param position to spin the flywheel
     */
    public ShooterHoodPositionTask(DigitalOperation position)
    {
        super(position, HOOD_POSITIONS, 0.1);
    }
}
