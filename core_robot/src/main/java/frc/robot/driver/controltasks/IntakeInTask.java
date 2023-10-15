package frc.robot.driver.controltasks;

import frc.robot.driver.*;

public class IntakeInTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.IntakeIn,
            DigitalOperation.IntakeOut,
            DigitalOperation.IntakeOutSlow,
            DigitalOperation.IntakeOutSuperSlow,
        };

    public IntakeInTask(boolean intakeIn)
    {
        super(
            intakeIn ? DigitalOperation.IntakeIn : DigitalOperation.IntakeOut,
            IntakeInTask.possibleOperations,
            true);
    }

    public IntakeInTask(boolean intakeIn, double timeout)
    {
        super(
            intakeIn ? DigitalOperation.IntakeIn : DigitalOperation.IntakeOut,
            IntakeInTask.possibleOperations,
            timeout);
    }
}
