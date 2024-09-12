package frc.robot.driver.controltasks;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IIMUManager;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.Pose2d;
import frc.lib.robotprovider.TrajectoryState;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.TankDriveTrainMechanism;
import frc.robot.mechanisms.PigeonManager;

/**
 * Task that follows a path
 * 
 */
public class FollowPathTask extends ControlTaskBase
{
    private final String pathName;

    protected ITimer timer;

    private IIMUManager positionManager;
    private TankDriveTrainMechanism driveTrain;

    private ITrajectory trajectory;
    private double duration;

    private double startTime;
    private double startLeftPosition;
    private double startRightPosition;
    private double startHeading;
    private double startXPosition;
    private double startYPosition;

    /**
     * Initializes a new FollowPathTask
     * @param pathName the path to follow
     */
    public FollowPathTask(String pathName)
    {
        this.pathName = pathName;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.startTime = this.timer.get();

        this.driveTrain = this.getInjector().getInstance(TankDriveTrainMechanism.class);
        this.startXPosition = this.driveTrain.getXPosition();
        this.startYPosition = this.driveTrain.getYPosition();
        this.startLeftPosition = this.driveTrain.getLeftPosition();
        this.startRightPosition = this.driveTrain.getRightPosition();

        this.positionManager = this.getInjector().getInstance(PigeonManager.class);
        this.startHeading = this.positionManager.getYaw();

        TrajectoryManager pathManager = this.getInjector().getInstance(TrajectoryManager.class);
        this.trajectory = pathManager.getTrajectory(this.pathName);
        if (this.trajectory == null)
        {
            ExceptionHelpers.Assert(false, "Unknown trajectory '" + this.pathName + "'");
            this.startTime = 0.0;
            return;
        }

        this.duration = this.trajectory.getDuration();

        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePathMode, true);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftPosition, this.startLeftPosition);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightPosition, this.startRightPosition);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainHeadingCorrection, 0.0);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        // double elapsedTime = this.timer.get() - this.startTime;
        // if (elapsedTime > this.duration)
        // {
        //     elapsedTime = this.duration;
        // }

        // double currentHeading = this.positionManager.getAngle();

        // TrajectoryState state = this.trajectory.get(elapsedTime);

        // double leftGoalPosition = step.getLeftPosition() * HardwareConstants.DRIVETRAIN_LEFT_TICKS_PER_INCH;
        // double rightGoalPosition = step.getRightPosition() * HardwareConstants.DRIVETRAIN_RIGHT_TICKS_PER_INCH;
        // this.setAnalogOperationState(AnalogOperation.DriveTrainLeftPosition, this.startLeftPosition + leftGoalPosition);
        // this.setAnalogOperationState(AnalogOperation.DriveTrainRightPosition, this.startRightPosition + rightGoalPosition);
        // this.setAnalogOperationState(AnalogOperation.DriveTrainLeftVelocity, step.getLeftVelocity());
        // this.setAnalogOperationState(AnalogOperation.DriveTrainRightVelocity, step.getRightVelocity());
        // this.setAnalogOperationState(AnalogOperation.DriveTrainHeadingCorrection, (this.startHeading + step.getHeading()) - currentHeading);
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        super.stop();

        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightPosition, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainHeadingCorrection, 0.0);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightPosition, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftVelocity, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightVelocity, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainHeadingCorrection, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.timer.get() - this.startTime >= this.duration;
    }
}
