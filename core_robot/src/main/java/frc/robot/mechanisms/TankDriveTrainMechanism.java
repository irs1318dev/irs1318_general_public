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
public class TankDriveTrainMechanism implements IMechanism
{
    private static final int pidSlotId = 0;
    private static final int FRAME_PERIOD_HZ = 200;
    private static final int FRAME_PERIOD_MS = 5;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final ITalonFX leftMotor;
    private final ITalonFX rightMotor;

    private PIDHandler leftPID;
    private PIDHandler rightPID;

    private boolean usePID;
    private boolean usePathMode;
    private boolean usePositionalMode;
    private boolean useBrakeMode;

    private double leftVelocity;
    private double leftError;
    private double leftPosition;
    private double rightVelocity;
    private double rightError;
    private double rightPosition;

    private double odometryX;
    private double odometryY;
    private double odometryAngle;
    private double prevLeftDistance;
    private double prevRightDistance;

    /**
     * Initializes a new DriveTrainMechanism
     * @param driver to use
     * @param logger to use
     * @param provider for obtaining electronics objects
     * @param timer to use
     */
    @Inject
    public TankDriveTrainMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        ITimer timer)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        this.leftMotor = provider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_LEFT_MASTER_CAN_ID);
        this.leftMotor.setMotorOutputSettings(HardwareConstants.TANK_DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT, MotorNeutralMode.Brake);
        this.leftMotor.setFeedbackUpdateRate(TankDriveTrainMechanism.FRAME_PERIOD_HZ);
        this.leftMotor.setErrorUpdateRate(TankDriveTrainMechanism.FRAME_PERIOD_HZ);
        this.leftMotor.setPIDF(
            TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KP,
            TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KI,
            TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KD,
            TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KF,
            TankDriveTrainMechanism.pidSlotId);
        this.leftMotor.setVoltageCompensation(
            TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.leftMotor.setCurrentLimit(
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        ITalonFX leftFollowerMotor1 = provider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_LEFT_FOLLOWER_CAN_ID);
        leftFollowerMotor1.setMotorOutputSettings(HardwareConstants.TANK_DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT, MotorNeutralMode.Brake);
        leftFollowerMotor1.follow(this.leftMotor);
        leftFollowerMotor1.setVoltageCompensation(
            TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        leftFollowerMotor1.setCurrentLimit(
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        this.rightMotor = provider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_RIGHT_MASTER_CAN_ID);
        this.rightMotor.setMotorOutputSettings(HardwareConstants.TANK_DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT, MotorNeutralMode.Brake);
        this.leftMotor.setFeedbackUpdateRate(TankDriveTrainMechanism.FRAME_PERIOD_HZ);
        this.leftMotor.setErrorUpdateRate(TankDriveTrainMechanism.FRAME_PERIOD_HZ);
        this.rightMotor.setPIDF(
            TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KP,
            TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KI,
            TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KD,
            TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KF,
            TankDriveTrainMechanism.pidSlotId);
        this.rightMotor.setVoltageCompensation(
            TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        this.rightMotor.setCurrentLimit(
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        ITalonFX rightFollowerMotor1 = provider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID);
        rightFollowerMotor1.setMotorOutputSettings(HardwareConstants.TANK_DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT, MotorNeutralMode.Brake);
        rightFollowerMotor1.follow(this.rightMotor);
        rightFollowerMotor1.setVoltageCompensation(
            TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED,
            TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        rightFollowerMotor1.setCurrentLimit(
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        this.leftPID = null;
        this.rightPID = null;

        this.usePID = TuningConstants.TANK_DRIVETRAIN_USE_PID;
        this.usePathMode = false;
        this.usePositionalMode = false;
        this.useBrakeMode = false;

        this.leftVelocity = 0.0;
        this.leftError = 0.0;
        this.leftPosition = 0.0;
        this.rightVelocity = 0.0;
        this.rightError = 0.0;
        this.rightPosition = 0.0;

        this.odometryX = 0.0;
        this.odometryY = 0.0;
        this.odometryAngle = 0.0;
        this.prevLeftDistance = 0.0;
        this.prevRightDistance = 0.0;

        this.setControlMode();
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

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.leftVelocity = this.leftMotor.getVelocity();
        this.rightVelocity = this.rightMotor.getVelocity();

        this.leftPosition = this.leftMotor.getPosition();
        this.rightPosition = this.rightMotor.getPosition();

        this.leftError = this.leftMotor.getError();
        this.rightError = this.rightMotor.getError();

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
        Setpoint setpoint;
        if (this.usePathMode)
        {
            setpoint = this.calculatePathModeSetpoint();
        }
        else if (this.usePositionalMode)
        {
            setpoint = this.calculatePositionModeSetpoint();
        }
        else
        {
            setpoint = this.calculateVelocityModeSetpoint();
        }

        double leftSetpoint = setpoint.getLeft();
        double rightSetpoint = setpoint.getRight();

        this.logger.logNumber(LoggingKey.DriveTrainLeftVelocityGoal, leftSetpoint);
        this.logger.logNumber(LoggingKey.DriveTrainRightVelocityGoal, rightSetpoint);

        // apply the setpoints to the motors
        this.leftMotor.set(leftSetpoint);
        this.rightMotor.set(rightSetpoint);
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.leftMotor.stop();
        this.rightMotor.stop();

        this.leftMotor.reset();
        this.rightMotor.reset();

        if (this.leftPID != null)
        {
            this.leftPID.reset();
        }

        if (this.rightPID != null)
        {
            this.rightPID.reset();
        }

        this.leftVelocity = 0.0;
        this.leftError = 0.0;
        this.leftPosition = 0;
        this.rightVelocity = 0.0;
        this.rightError = 0.0;
        this.rightPosition = 0;
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
        TalonFXControlMode mode = TalonFXControlMode.PercentOutput;
        if (this.usePID)
        {
            if (this.usePathMode)
            {
                this.leftPID = new PIDHandler(
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KP,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KI,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KD,
                    TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KF,
                    1.0,
                    -TuningConstants.TANK_DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    TuningConstants.TANK_DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    this.timer);
                this.rightPID = new PIDHandler(
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
                    this.leftPID = new PIDHandler(
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_LEFT_KP,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_LEFT_KI,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_LEFT_KD,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_PID_LEFT_KF,
                        1.0,
                        -TuningConstants.TANK_DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        TuningConstants.TANK_DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        this.timer);
                    this.rightPID = new PIDHandler(
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
                    this.leftPID = new PIDHandler(
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KP,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KI,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KD,
                        TuningConstants.TANK_DRIVETRAIN_POSITION_PID_LEFT_KF,
                        1.0,
                        -TuningConstants.TANK_DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                        TuningConstants.TANK_DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                        this.timer);
                    this.rightPID = new PIDHandler(
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
                this.leftPID = null;
                this.rightPID = null;
            }

            mode = TalonFXControlMode.Velocity;
            this.leftMotor.setSelectedSlot(TankDriveTrainMechanism.pidSlotId);
            this.rightMotor.setSelectedSlot(TankDriveTrainMechanism.pidSlotId);
        }

        this.leftMotor.setControlMode(mode);
        this.rightMotor.setControlMode(mode);
    }

    /**
     * Calculate the setting to use based on the inputs when in velocity mode
     * @return settings for left and right motor
     */
    private Setpoint calculateVelocityModeSetpoint()
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
        double left = this.applyPowerLevelRange(leftVelocityGoal);
        double right = this.applyPowerLevelRange(rightVelocityGoal);

        this.assertPowerLevelRange(left, "left");
        this.assertPowerLevelRange(right, "right");

        // if we are using PID, then we base the setpoint on the max velocity
        if (this.usePID)
        {
            left *= TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KS;
            right *= TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KS;
        }

        return new Setpoint(left, right);
    }

    /**
     * Calculate the setting to use based on the inputs when in path mode
     * @return settings for left and right motor
     */
    private Setpoint calculatePathModeSetpoint()
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
        double leftGoal = this.leftPID.calculatePosition(leftPositionGoal, this.leftPosition);
        double rightGoal = this.rightPID.calculatePosition(rightPositionGoal, this.rightPosition);

        // add in velocity as a type of feed-forward
        leftGoal += leftVelocityGoal * TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KV;
        rightGoal += rightVelocityGoal * TuningConstants.TANK_DRIVETRAIN_PATH_PID_RIGHT_KV;

        // apply cross-coupling changes
        double leftPositionError = this.leftPID.getError();
        double rightPositionError = this.rightPID.getError();

        double positionErrorMagnitudeDelta = leftPositionError - rightPositionError;
        if (TuningConstants.TANK_DRIVETRAIN_USE_CROSS_COUPLING
            && !Helpers.WithinDelta(positionErrorMagnitudeDelta, 0.0, TuningConstants.TANK_DRIVETRAIN_CROSS_COUPLING_ZERO_ERROR_RANGE))
        {
            // add the delta times the coupling factor to the left, and subtract from the right
            // (if left error is greater than right error, left should be given some more power than right)
            leftGoal += TuningConstants.TANK_DRIVETRAIN_PATH_PID_LEFT_KCC * positionErrorMagnitudeDelta;
            rightGoal -= TuningConstants.TANK_DRIVETRAIN_PATH_PID_RIGHT_KCC * positionErrorMagnitudeDelta;
        }

        // apply heading correction
        if (TuningConstants.TANK_DRIVETRAIN_USE_HEADING_CORRECTION
            && headingCorrection != 0.0)
        {
            leftGoal += TuningConstants.TANK_DRIVETRAIN_PATH_LEFT_HEADING_CORRECTION * headingCorrection;
            rightGoal -= TuningConstants.TANK_DRIVETRAIN_PATH_RIGHT_HEADING_CORRECTION * headingCorrection;
        }

        // velocity plus position correction could put us over our max or under our min power levels
        leftGoal = this.applyPowerLevelRange(leftGoal);
        rightGoal = this.applyPowerLevelRange(rightGoal);

        this.assertPowerLevelRange(leftGoal, "left velocity (goal)");
        this.assertPowerLevelRange(rightGoal, "right velocity (goal)");

        if (this.usePID)
        {
            leftGoal *= TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KS;
            rightGoal *= TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KS;
        }

        return new Setpoint(leftGoal, rightGoal);
    }

    /**
     * Calculate the setting to use based on the inputs when in position mode
     * @return settings for left and right motor
     */
    private Setpoint calculatePositionModeSetpoint()
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
            leftPower = this.leftPID.calculatePosition(leftPositionGoal, this.leftPosition);
            rightPower = this.rightPID.calculatePosition(rightPositionGoal, this.rightPosition);

            // apply cross-coupling changes
            double leftPositionError = this.leftPID.getError();
            double rightPositionError = this.rightPID.getError();

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
            leftPower *= TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KS;
            rightPower *= TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KS;
        }

        return new Setpoint(leftPower, rightPower);
    }

    /**
     * Assert that the power level is within the required range
     * @param powerLevel to verify
     * @param side indicator for the exception message if incorrect
     */
    private void assertPowerLevelRange(double powerLevel, String side)
    {
        if (powerLevel < TankDriveTrainMechanism.POWERLEVEL_MIN)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too low!");
            }

            return;
        }

        if (powerLevel > TankDriveTrainMechanism.POWERLEVEL_MAX)
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
        return Helpers.EnforceRange(powerLevel, TankDriveTrainMechanism.POWERLEVEL_MIN, TankDriveTrainMechanism.POWERLEVEL_MAX);
    }

    /**
     * Simple holder of setpoint information for the left and right sides
     */
    private class Setpoint
    {
        private double left;
        private double right;

        /**
         * Initializes a new Setpoint
         * @param left value to apply
         * @param right value to apply
         */
        public Setpoint(double left, double right)
        {
            this.left = left;
            this.right = right;
        }

        /**
         * gets the left setpoint
         * @return left setpoint value
         */
        public double getLeft()
        {
            return this.left;
        }

        /**
         * gets the right setpoint
         * @return right setpoint value
         */
        public double getRight()
        {
            return this.right;
        }
    }
}
