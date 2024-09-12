package frc.robot.driver.controltasks;

import frc.robot.driver.*;

/**
 * Task that applies the starting angle
 * 
 */
public class PositionStartingTask extends UpdateCycleTask
{
    private final Double orientationAngle;
    private final boolean resetDriveTrain;
    private final boolean resetOrientation;
    /**
     * Initializes a new PositionStartingTask
     * @param resetDriveTrain - whether to reset the drivetrain wheels (to read from the absolute encoders)
     * @param resetOrientation - whether to reset the orientation of the robot
     */
    public PositionStartingTask(boolean resetDriveTrain, boolean resetOrientation)
    {
        super(1);

        this.orientationAngle = null;
        this.resetDriveTrain = resetDriveTrain;
        this.resetOrientation = resetOrientation;
    }

    /**
     * Initializes a new PositionStartingTask
     * @param orientationAngle - offset to use from the default of facing away from the alliance driver station (in degrees)
     * @param resetDriveTrain - whether to reset the drivetrain wheels (to read from the absolute encoders)
     * @param resetOrientation - whether to reset the orientation of the robot
     */
    public PositionStartingTask(double orientationAngle, boolean resetDriveTrain, boolean resetOrientation)
    {
        super(1);

        this.orientationAngle = orientationAngle;
        this.resetDriveTrain = resetDriveTrain;
        this.resetOrientation = resetOrientation;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();

        this.setEverything();
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     */
    @Override
    public void update()
    {
        super.update();

        this.setEverything();
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.PositionStartingAngle, 0.0);
        this.setDigitalOperationState(DigitalOperation.DriveTrainReset, false);
        this.setDigitalOperationState(DigitalOperation.PositionResetFieldOrientation, false);
    }

    private void setEverything()
    {
        if (this.orientationAngle != null)
        {
            this.setAnalogOperationState(AnalogOperation.PositionStartingAngle, this.orientationAngle);
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainReset, this.resetDriveTrain);
        this.setDigitalOperationState(DigitalOperation.PositionResetFieldOrientation, this.resetOrientation);
    }
}
