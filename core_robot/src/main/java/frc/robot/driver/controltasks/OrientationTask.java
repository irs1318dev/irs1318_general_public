package frc.robot.driver.controltasks;

import frc.lib.controllers.PIDHandler;
import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.ITimer;
import frc.robot.TuningConstants;
import frc.robot.driver.*;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PigeonManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class OrientationTask extends ControlTaskBase
{
    private final boolean useTime;
    private final double turnAngle;
    private final double waitTime;
    private final boolean relativeMode;
    private final boolean fastMode;

    private PIDHandler turnPidHandler;
    private PigeonManager pManager;
    private DriveTrainMechanism dt;
    private ITimer timer;

    private double desiredTurnVelocity;
    private Double completeTime;
    private double startingAngle;

    /**
    * Initializes a new OrientationTask using time to make sure we completed turn
    * @param turnAngle the desired angle
    */
    public OrientationTask(double turnAngle)
    {
        this(true, turnAngle);
    }

    /**
    * Initializes a new OrientationTask
    * @param useTime whether to make sure we completed turn for a second or not
    * @param turnAngle the desired angle
    */
    public OrientationTask(boolean useTime, double turnAngle)
    {
        this(
            useTime,
            turnAngle,
            TuningConstants.NAVX_TURN_COMPLETE_TIME,
            false,
            false);
    }

    /**
     * Initializes a new OrientationTask using a variable wait time after turn has reached the goal
     * @param turnAngle the desired angle
     * @param waitTime the desired wait time
     */
    public OrientationTask(double turnAngle, double waitTime)
    {
        this(
            true,
            turnAngle,
            waitTime,
            false,
            false);
    }

    /**
     * Initializes a new OrientationTask
     * @param useTime whether to make sure we completed turn for a second or not
     * @param turnAngle the desired angle
     * @param waitTime the desired wait time
     * @param relativeMode whether to use relative mode (turn relative to current angle), or absolute mode (turn relative to starting orientation)
     * @param fastMode whether to use fast mode (or slow/consistent mode)
     */
    public OrientationTask(boolean useTime, double turnAngle, double waitTime, boolean relativeMode, boolean fastMode)
    {
        this.useTime = useTime;
        this.turnAngle = turnAngle;
        this.waitTime = waitTime;
        this.relativeMode = relativeMode;
        this.fastMode = fastMode;

        this.startingAngle = 0;
        this.turnPidHandler = null;
        this.completeTime = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.pManager = this.getInjector().getInstance(PigeonManager.class);
        this.dt = this.getInjector().getInstance(DriveTrainMechanism.class);
        this.timer = this.getInjector().getInstance(ITimer.class);

        this.turnPidHandler = this.createTurnHandler();
        if (this.relativeMode)
        {
            this.startingAngle = this.pManager.getYaw();

            // if the navx isn't working, let's fall back to using odometry
            if (!this.pManager.getIsConnected())
            {
                ////this.startingAngle = this.dt.getOdometryAngle();
            }
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainSimpleMode, this.fastMode);

        double currentMeasuredAngle = this.pManager.getYaw();
        double currentDesiredAngle = this.turnAngle + this.startingAngle;

        // if the navx isn't connected, let's fall back to using odometry
        if (!this.pManager.getIsConnected())
        {
            ////currentMeasuredAngle = this.dt.getOdometryAngle();
            currentDesiredAngle = currentDesiredAngle % 360.0; // odometry measures angles from 0 to 30 only
        }

        this.desiredTurnVelocity = -1.0 * this.turnPidHandler.calculatePosition(currentDesiredAngle, currentMeasuredAngle);

        this.setAnalogOperationState(AnalogOperation.DriveTrainTurn, this.desiredTurnVelocity);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainSimpleMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurn, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        double currentMeasuredAngle = this.pManager.getYaw();
        double currentTurnVelocity = this.dt.getLeftVelocity();
        double currentDesiredAngle = this.startingAngle + this.turnAngle;

        // if the navx isn't connected, let's fall back to using odometry
        if (!this.pManager.getIsConnected())
        {
            ////currentMeasuredAngle = this.dt.getOdometryAngle();
            currentDesiredAngle = currentDesiredAngle % 360.0;
        }

        if (this.fastMode && this.useTime)
        {
            // check for timeout if we are in fast mode
            if (this.completeTime == null)
            {
                this.completeTime = timer.get();
            }
            else if (this.timer.get() - this.completeTime >= this.waitTime)
            {
                return true;
            }
        }

        double centerAngleDifference = Math.abs(currentMeasuredAngle - currentDesiredAngle);
        if ((!this.fastMode && centerAngleDifference > TuningConstants.MAX_NAVX_TURN_RANGE_DEGREES) ||
            (this.fastMode && centerAngleDifference > TuningConstants.MAX_NAVX_FAST_TURN_RANGE_DEGREES))
        {
            return false;
        }

        if (!this.fastMode && !this.useTime)
        {
            return true;
        }
        else
        {
            // If desired and current turn velocity are near 0, complete this task. Otherwise, use timer.
            if (Helpers.WithinDelta(currentTurnVelocity, 0.0, TuningConstants.NAVX_TURN_COMPLETE_CURRENT_VELOCITY_DELTA)
                && Helpers.WithinDelta(this.desiredTurnVelocity, 0.0, TuningConstants.NAVX_TURN_COMPLETE_DESIRED_VELOCITY_DELTA))
            {
                return true;
            }

            if (this.completeTime == null)
            {
                this.completeTime = this.timer.get();
                return false;
            }
            else if (this.timer.get() - this.completeTime < this.waitTime)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KP : TuningConstants.NAVX_TURN_PID_KP,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KI : TuningConstants.NAVX_TURN_PID_KI,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KD : TuningConstants.NAVX_TURN_PID_KD,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KF : TuningConstants.NAVX_TURN_PID_KF,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_KS : TuningConstants.NAVX_TURN_PID_KS,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_MIN : TuningConstants.NAVX_TURN_PID_MIN,
            this.fastMode ? TuningConstants.NAVX_FAST_TURN_PID_MAX : TuningConstants.NAVX_TURN_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}
