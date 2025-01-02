package frc.robot.mechanisms;

import javax.inject.Singleton;

import frc.robot.*;
import frc.lib.*;
import frc.lib.controllers.PIDHandler;
import frc.lib.driver.*;
import frc.lib.filters.*;
import frc.lib.helpers.AnglePair;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.mechanisms.PowerManager.CurrentLimiting;

import com.google.inject.Inject;

/**
 * Drivetrain mechanism.
 * The mechanism defines the logic that controls a mechanism given inputs and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs.
 * 
 */
@Singleton
public class SimpleTankDriveTrainMechanism implements IMechanism
{
    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final IMotor leftMotor;
    private final IMotor rightMotor;

    private final IEncoder leftEncoder;
    private final IEncoder rightEncoder;

    private boolean usePID;
    private PIDHandler leftPID;
    private PIDHandler rightPID;

    private PIDHandler leftExtraPID;
    private PIDHandler rightExtraPID;

    private double leftVelocity;
    private double rightVelocity;
    private int leftPosition;
    private int rightPosition;
    private double leftError;
    private double rightError;

    private double odometryX;
    private double odometryY;
    private double odometryAngle;

    private double prevLeftDistance;
    private double prevRightDistance;
    private boolean usePathMode;
    private boolean usePositionalMode;
    private boolean useBrakeMode;

    /**
     * Initializes a new DriveTrainMechanism
     * @param driver to use
     * @param logger to use
     * @param provider for obtaining electronics objects
     * @param timer to use
     */
    @Inject
    public SimpleTankDriveTrainMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        this.leftMotor = provider.getVictor(
            ElectronicsConstants.SIMPLETANK_DRIVETRAIN_LEFT_TALON_CHANNEL);

        this.rightMotor = provider.getVictor(
            ElectronicsConstants.SIMPLETANK_DRIVETRAIN_RIGHT_TALON_CHANNEL);

        this.leftEncoder = provider.getEncoder(
            ElectronicsConstants.SIMPLETANK_DRIVETRAIN_LEFT_ENCODER_CHANNEL_A,
            ElectronicsConstants.SIMPLETANK_DRIVETRAIN_LEFT_ENCODER_CHANNEL_B);

        this.rightEncoder = provider.getEncoder(
            ElectronicsConstants.SIMPLETANK_DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A,
            ElectronicsConstants.SIMPLETANK_DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B);

        this.leftEncoder.setDistancePerPulse(HardwareConstants.TANK_DRIVETRAIN_LEFT_PULSE_DISTANCE);
        this.rightEncoder.setDistancePerPulse(HardwareConstants.TANK_DRIVETRAIN_RIGHT_PULSE_DISTANCE);

        this.usePID = TuningConstants.TANK_DRIVETRAIN_USE_PID;
        this.setControlMode();
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.leftVelocity = this.leftEncoder.getRate();
        this.rightVelocity = this.rightEncoder.getRate();

        this.leftPosition = this.leftEncoder.get();
        this.rightPosition = this.rightEncoder.get();

        if (this.leftPID != null && this.rightPID != null)
        {
            this.leftError = this.leftPID.getError();
            this.rightError = this.rightPID.getError();
        }
        else
        {
            this.leftError = 0.0;
            this.rightError = 0.0;
        }

        this.logger.logNumber(LoggingKey.DriveTrainLeftVelocity, this.leftVelocity);
        this.logger.logNumber(LoggingKey.DriveTrainLeftError, this.leftError);
        this.logger.logNumber(LoggingKey.DriveTrainLeftTicks, this.leftPosition);
        this.logger.logNumber(LoggingKey.DriveTrainRightVelocity, this.rightVelocity);
        this.logger.logNumber(LoggingKey.DriveTrainRightError, this.rightError);
        this.logger.logNumber(LoggingKey.DriveTrainRightTicks, this.rightPosition);

        // Calculate odometry:
        // check the current distance recorded by the encoders
        double leftDistance = this.leftPosition * HardwareConstants.TANK_DRIVETRAIN_LEFT_PULSE_DISTANCE;
        double rightDistance = this.rightPosition * HardwareConstants.TANK_DRIVETRAIN_RIGHT_PULSE_DISTANCE;

        // calculate the angle (in radians) based on the total distance traveled
        double angleR = (leftDistance - rightDistance) / HardwareConstants.TANK_DRIVETRAIN_WHEEL_SEPARATION_DISTANCE;

        // correct for odometry angle inconsistencies
        angleR *= TuningConstants.TANK_DRIVETRAIN_ENCODER_ODOMETRY_ANGLE_CORRECTION;

        // calculate the average distance traveled
        double averagePositionChange = ((leftDistance - this.prevLeftDistance) + (rightDistance - this.prevRightDistance)) / 2;

        // calculate the change since last time, and update our relative position
        this.odometryX += averagePositionChange * Math.cos(angleR);
        this.odometryY += averagePositionChange * Math.sin(angleR);

        this.odometryAngle = (angleR * 360.0 / (2.0 * Math.PI)) % 360;

        // record distance for next time
        this.prevLeftDistance = leftDistance;
        this.prevRightDistance = rightDistance;

        this.logger.logNumber(LoggingKey.DriveTrainXPosition, this.odometryX);
        this.logger.logNumber(LoggingKey.DriveTrainYPosition, this.odometryY);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update(RobotMode mode)
    {
        if (this.driver.getDigital(DigitalOperation.DriveTrainEnablePID))
        {
            this.usePID = true;
            this.setControlMode();
        }
        else if (this.driver.getDigital(DigitalOperation.DriveTrainDisablePID))
        {
            this.usePID = false;
            this.setControlMode();
        }

        // check our desired PID mode (needed for positional mode or break mode)
        boolean newUsePathMode = this.driver.getDigital(DigitalOperation.DriveTrainUsePathMode);
        boolean newUsePositionalMode = this.driver.getDigital(DigitalOperation.DriveTrainUsePositionalMode);
        boolean newUseBrakeMode = this.driver.getDigital(DigitalOperation.DriveTrainUseBrakeMode);
        if (newUsePathMode != this.usePathMode ||
            newUsePositionalMode != this.usePositionalMode ||
            newUseBrakeMode != this.useBrakeMode)
        {
            this.usePathMode = newUsePathMode;
            this.usePositionalMode = newUsePositionalMode;
            this.useBrakeMode = newUseBrakeMode;

            // re-create PID handler
            this.setControlMode();
        }

        // calculate desired setting for the current mode
        Output output;
        if (this.usePathMode)
        {
            output = this.calculatePathModeOutput();
        }
        else if (this.usePositionalMode)
        {
            output = this.calculatePositionModeOutput();
        }
        else
        {
            output = this.calculateVelocityModeOutput();
        }

        double leftOutput = output.getLeft();
        double rightOutput = -output.getRight(); // right motor faces "backwards"

        this.logger.logNumber(LoggingKey.DriveTrainLeftMotorOut, leftOutput);
        this.logger.logNumber(LoggingKey.DriveTrainRightMotorOut, rightOutput);

        // apply the outputs to the motors
        this.leftMotor.set(leftOutput);
        this.rightMotor.set(rightOutput);
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.leftMotor.set(0.0);
        this.rightMotor.set(0.0);

        this.leftEncoder.reset();
        this.rightEncoder.reset();

        if (this.leftPID != null && this.rightPID != null)
        {
            this.leftPID.reset();
            this.rightPID.reset();
        }
        else
        {
            this.leftPID = null;
            this.rightPID = null;
        }

        if (this.leftExtraPID != null && this.rightExtraPID != null)
        {
            this.leftExtraPID.reset();
            this.rightExtraPID.reset();
        }
        else
        {
            this.leftExtraPID = null;
            this.rightExtraPID = null;
        }

        this.leftVelocity = 0.0;
        this.leftError = 0.0;
        this.leftPosition = 0;
        this.rightVelocity = 0.0;
        this.rightError = 0.0;
        this.rightPosition = 0;
    }

    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getLeftVelocity()
    {
        return this.leftVelocity;
    }

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getRightVelocity()
    {
        return this.rightVelocity;
    }

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getLeftError()
    {
        return this.leftError;
    }

    /**
     * get the distance from the right encoder
     * @return a value indicating the distance
     */
    public double getRightError()
    {
        return this.rightError;
    }

    /**
     * get the ticks from the left encoder
     * @return a value indicating the position we are at
     */
    public double getLeftPosition()
    {
        return this.leftPosition;
    }

    /**
     * get the ticks from the right encoder
     * @return a value indicating the position we are at
     */
    public double getRightPosition()
    {
        return this.rightPosition;
    }

    public double getXPosition()
    {
        return this.odometryX;
    }

    public double getYPosition()
    {
        return this.odometryY;
    }

    /**
     * create a PIDHandler based on our current settings
     */
    private void setControlMode()
    {
        if (this.usePID)
        {
            this.leftPID = new PIDHandler(
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KP,
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KI,
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KD,
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KF,
                1.0,
                -TuningConstants.TANK_DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                this.timer);
            this.rightPID = new PIDHandler(
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KP,
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KI,
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KD,
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KF,
                1.0,
                -TuningConstants.TANK_DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                TuningConstants.TANK_DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                this.timer);

            if (this.usePathMode)
            {
                this.leftExtraPID = new PIDHandler(
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KP,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KI,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KD,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KF,
                    1.0,
                    -TuningConstants.TANK_DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    TuningConstants.TANK_DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    this.timer);
                this.rightExtraPID = new PIDHandler(
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_RIGHT_KP,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_RIGHT_KI,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_RIGHT_KD,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_RIGHT_KF,
                    1.0,
                    -TuningConstants.TANK_DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    TuningConstants.TANK_DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    this.timer);
            }
            else if (this.usePositionalMode)
            {
                if (this.useBrakeMode)
                {
                    this.leftExtraPID = new PIDHandler(
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_LEFT_KP,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_LEFT_KI,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_LEFT_KD,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_LEFT_KF,
                        1.0,
                        -TuningConstants.TANK_DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        this.timer);
                    this.rightExtraPID = new PIDHandler(
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_RIGHT_KP,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_RIGHT_KI,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_RIGHT_KD,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_RIGHT_KF,
                        1.0,
                        -TuningConstants.TANK_DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        this.timer);
                }
                else
                {
                    this.leftExtraPID = new PIDHandler(
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KP,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KI,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KD,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KF,
                        1.0,
                        -TuningConstants.TANK_DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                        TuningConstants.TANK_DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                        this.timer);
                    this.rightExtraPID = new PIDHandler(
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_RIGHT_KP,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_RIGHT_KI,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_RIGHT_KD,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_RIGHT_KF,
                        1.0,
                        -TuningConstants.TANK_DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                        TuningConstants.TANK_DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                        this.timer);
                }
            }
            else
            {
                this.leftExtraPID = null;
                this.rightExtraPID = null;
            }
        }
        else
        {
            this.leftPID = null;
            this.rightPID = null;

            this.leftExtraPID = null;
            this.rightExtraPID = null;
        }
    }

    /**
     * Calculate the setting to use based on the inputs when in velocity mode
     * @return settings for left and right motor
     */
    private Output calculateVelocityModeOutput()
    {
        // velocity goals represent the desired percentage of the max velocity
        double leftVelocityGoal = 0.0;
        double rightVelocityGoal = 0.0;

        // get a value indicating that we should be in simple mode...
        boolean simpleDriveModeEnabled = this.driver.getDigital(DigitalOperation.DriveTrainSimpleMode);

        // get the X and Y values from the operator.  We expect these to be between -1.0 and 1.0,
        // with this value representing the forward velocity percentage and right turn percentage (of max speed)
        double turnAmount = this.driver.getAnalog(AnalogOperation.DriveTrainTurn);
        double forwardVelocity = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

        // Negate the x and y if DriveTrainSwapFrontOrientation is true
        if (this.driver.getDigital(DigitalOperation.DriveTrainSwapFrontOrientation))
        {
            turnAmount *= -1.0;
            forwardVelocity *= -1.0;
        }

        if ((!simpleDriveModeEnabled && TuningConstants.TANK_DRIVETRAIN_REGULAR_MODE_SQUARING)
            || (simpleDriveModeEnabled && TuningConstants.TANK_DRIVETRAIN_SIMPLE_MODE_SQUARING))
        {
            if (turnAmount >= 0)
            {
                turnAmount = turnAmount * turnAmount;
            }
            else
            {
                turnAmount = -1.0 * turnAmount * turnAmount;
            }

            if (forwardVelocity >= 0)
            {
                forwardVelocity = forwardVelocity * forwardVelocity;
            }
            else
            {
                forwardVelocity = -1.0 * forwardVelocity * forwardVelocity;
            }
        }

        // adjust the intensity of the input
        if (simpleDriveModeEnabled)
        {
            if (Math.abs(forwardVelocity) < Math.abs(turnAmount))
            {
                // in-place turn
                leftVelocityGoal = turnAmount;
                rightVelocityGoal = -turnAmount;
            }
            else
            {
                // forward/backward
                leftVelocityGoal = forwardVelocity;
                rightVelocityGoal = forwardVelocity;
            }
        }
        else
        {
            leftVelocityGoal = (TuningConstants.TANK_DRIVETRAIN_K1 * forwardVelocity) + (TuningConstants.TANK_DRIVETRAIN_K2 * turnAmount);
            rightVelocityGoal = (TuningConstants.TANK_DRIVETRAIN_K1 * forwardVelocity) + (-TuningConstants.TANK_DRIVETRAIN_K2 * turnAmount);
        }

        // decrease the desired velocity based on the configured max power level
        leftVelocityGoal = leftVelocityGoal * TuningConstants.TANK_DRIVETRAIN_MAX_POWER_LEVEL;
        rightVelocityGoal = rightVelocityGoal * TuningConstants.TANK_DRIVETRAIN_MAX_POWER_LEVEL;

        // ensure that we don't give values outside the appropriate range
        double leftPower = this.applyPowerLevelRange(leftVelocityGoal);
        double rightPower = this.applyPowerLevelRange(rightVelocityGoal);

        this.assertPowerLevelRange(leftPower, "left");
        this.assertPowerLevelRange(rightPower, "right");

        // if we are using PID, then we base the setpoint on the max velocity
        if (this.usePID)
        {
            leftPower = this.leftPID.calculateVelocity(leftPower * TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KS, this.leftVelocity);
            rightPower = this.rightPID.calculateVelocity(rightPower * TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KS, this.rightVelocity);
        }

        return new Output(leftPower, rightPower);
    }

    /**
     * Calculate the setting to use based on the inputs when in path mode
     * @return settings for left and right motor
     */
    private Output calculatePathModeOutput()
    {
        // get the desired left and right values from the driver.
        // note that position goals are in inches and velocity goals are in inches/second
        double leftPositionGoal = this.driver.getAnalog(AnalogOperation.DriveTrainLeftPosition);
        double rightPositionGoal = this.driver.getAnalog(AnalogOperation.DriveTrainRightPosition);
        double leftVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainLeftVelocity);
        double rightVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainRightVelocity);
        double headingCorrection = this.driver.getAnalog(AnalogOperation.DriveTrainHeadingCorrection);

        leftVelocityGoal /= TuningConstants.TANK_DRIVETRAIN_PATH_LEFT_MAX_VELOCITY_INCHES_PER_SECOND;
        rightVelocityGoal /= TuningConstants.TANK_DRIVETRAIN_PATH_RIGHT_MAX_VELOCITY_INCHES_PER_SECOND;

        this.logger.logNumber(LoggingKey.DriveTrainLeftPositionGoal, leftPositionGoal);
        this.logger.logNumber(LoggingKey.DriveTrainRightPositionGoal, rightPositionGoal);
        this.logger.logNumber(LoggingKey.DriveTrainLeftVelocityGoal, leftVelocityGoal);
        this.logger.logNumber(LoggingKey.DriveTrainRightVelocityGoal, rightVelocityGoal);

        // use positional PID to get the relevant value
        double leftGoal = this.leftExtraPID.calculatePosition(leftPositionGoal, this.leftPosition);
        double rightGoal = this.rightExtraPID.calculatePosition(rightPositionGoal, this.rightPosition);

        // add in velocity as a type of feed-forward
        leftGoal += leftVelocityGoal * TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KV;
        rightGoal += rightVelocityGoal * TuningConstants.TANK_DRIVETRAIN_PATH_PID_RIGHT_KV;

        // apply cross-coupling changes
        double leftPositionError = this.leftExtraPID.getError();
        double rightPositionError = this.rightExtraPID.getError();

        double positionErrorMagnitudeDelta = leftPositionError - rightPositionError;
        if (TuningConstants.TANK_DRIVETRAIN_USE_CROSS_COUPLING &&
            !Helpers.WithinDelta(positionErrorMagnitudeDelta, 0.0, TuningConstants.TANK_DRIVETRAIN_CROSS_COUPLING_ZERO_ERROR_RANGE))
        {
            // add the delta times the coupling factor to the left, and subtract from the right
            // (if left error is greater than right error, left should be given some more power than right)
            leftGoal += TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KCC * positionErrorMagnitudeDelta;
            rightGoal -= TuningConstants.TANK_DRIVETRAIN_PATH_PID_RIGHT_KCC * positionErrorMagnitudeDelta;
        }

        // apply heading correction
        if (TuningConstants.TANK_DRIVETRAIN_USE_HEADING_CORRECTION && headingCorrection != 0.0)
        {
            leftGoal += TuningConstants.TANK_DRIVETRAIN_PATH_LEFT_HEADING_CORRECTION * headingCorrection;
            rightGoal -= TuningConstants.TANK_DRIVETRAIN_PATH_RIGHT_HEADING_CORRECTION * headingCorrection;
        }

        // velocity plus position correction could put us over our max or under our min power levels
        leftGoal = this.applyPowerLevelRange(leftGoal);
        rightGoal = this.applyPowerLevelRange(rightGoal);

        this.assertPowerLevelRange(leftGoal, "left velocity (goal)");
        this.assertPowerLevelRange(rightGoal, "right velocity (goal)");

        leftGoal = this.leftPID.calculateVelocity(leftGoal * TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KS, this.leftVelocity);
        rightGoal = this.rightPID.calculateVelocity(rightGoal * TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KS, this.rightVelocity);

        return new Output(leftGoal, rightGoal);
    }

    /**
     * Calculate the setting to use based on the inputs when in position mode
     * @return settings for left and right motor
     */
    private Output calculatePositionModeOutput()
    {
        // get the desired left and right values from the driver.
        double leftPositionGoal = this.driver.getAnalog(AnalogOperation.DriveTrainLeftPosition);
        double rightPositionGoal = this.driver.getAnalog(AnalogOperation.DriveTrainRightPosition);

        this.logger.logNumber(LoggingKey.DriveTrainLeftPositionGoal, leftPositionGoal);
        this.logger.logNumber(LoggingKey.DriveTrainRightPositionGoal, rightPositionGoal);

        double leftPower;
        double rightPower;
        if (this.usePID)
        {
            // use positional PID to get the relevant value
            leftPower = this.leftExtraPID.calculatePosition(leftPositionGoal, this.leftPosition);
            rightPower = this.rightExtraPID.calculatePosition(rightPositionGoal, this.rightPosition);

            // apply cross-coupling changes
            double leftPositionError = this.leftExtraPID.getError();
            double rightPositionError = this.rightExtraPID.getError();

            double positionErrorMagnitudeDelta = leftPositionError - rightPositionError;
            if (TuningConstants.TANK_DRIVETRAIN_USE_CROSS_COUPLING
                && !Helpers.WithinDelta(positionErrorMagnitudeDelta, 0.0, TuningConstants.TANK_DRIVETRAIN_CROSS_COUPLING_ZERO_ERROR_RANGE))
            {
                // add the delta times the coupling factor to the left, and subtract from the right
                // (if left error is greater than right error, left should be given some more power than right)
                leftPower += TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KCC * positionErrorMagnitudeDelta;
                rightPower -= TuningConstants.TANK_DRIVETRAIN_POSITION_PID_RIGHT_KCC * positionErrorMagnitudeDelta;

                // cross-coupling could put us over our max or under our min power levels
                leftPower = this.applyPowerLevelRange(leftPower);
                rightPower = this.applyPowerLevelRange(rightPower);
            }
        }
        else
        {
            // calculate a desired power level
            leftPower = leftPositionGoal - this.leftPosition;
            rightPower = rightPositionGoal - this.rightPosition;
            if (Math.abs(leftPower) < 0.1)
            {
                leftPower = 0.0;
            }

            if (Math.abs(rightPower) < 0.1)
            {
                rightPower = 0.0;
            }

            leftPower *= TuningConstants.TANK_DRIVETRAIN_LEFT_POSITIONAL_NON_PID_MULTIPLICAND;
            rightPower *= TuningConstants.TANK_DRIVETRAIN_RIGHT_POSITIONAL_NON_PID_MULTIPLICAND;

            // ensure that we are within our power level range, and then scale it down
            leftPower = this.applyPowerLevelRange(leftPower) * TuningConstants.TANK_DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
            rightPower = this.applyPowerLevelRange(rightPower) * TuningConstants.TANK_DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
        }

        this.assertPowerLevelRange(leftPower, "left velocity (goal)");
        this.assertPowerLevelRange(rightPower, "right velocity (goal)");

        if (this.usePID)
        {
            leftPower = this.leftPID.calculateVelocity(leftPower * TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KS, this.leftVelocity);
            rightPower = this.rightPID.calculateVelocity(rightPower * TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KS, this.rightVelocity);
        }

        return new Output(leftPower, rightPower);
    }

    /**
     * Assert that the power level is within the required range
     * @param powerLevel to verify
     * @param side indicator for the exception message if incorrect
     */
    private void assertPowerLevelRange(double powerLevel, String side)
    {
        if (powerLevel < SimpleTankDriveTrainMechanism.POWERLEVEL_MIN)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too low!");
            }

            return;
        }

        if (powerLevel > SimpleTankDriveTrainMechanism.POWERLEVEL_MAX)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too high!");
            }

            return;
        }
    }

    /**
     * Reset the power level to be within the required range
     * @param powerLevel to reset
     * @return power level
     */
    private double applyPowerLevelRange(double powerLevel)
    {
        return Helpers.EnforceRange(powerLevel, SimpleTankDriveTrainMechanism.POWERLEVEL_MIN, SimpleTankDriveTrainMechanism.POWERLEVEL_MAX);
    }

    /**
     * Simple holder of motor output percentage information for the left and right sides
     */
    private class Output
    {
        private double left;
        private double right;

        /**
         * Initializes a new Output
         * @param left value to apply
         * @param right value to apply
         */
        public Output(double left, double right)
        {
            this.left = left;
            this.right = right;
        }

        /**
         * gets the left output
         * @return left output value
         */
        public double getLeft()
        {
            return this.left;
        }

        /**
         * gets the right output
         * @return right output value
         */
        public double getRight()
        {
            return this.right;
        }
    }
}
