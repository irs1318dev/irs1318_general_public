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
    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final IMotor leftMotor;
    private final IMotor rightMotor;

    /**
     * Initializes a new SimpleTankDriveTrainMechanism
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

        this.leftMotor = provider.getVictor(ElectronicsConstants.SIMPLE_TANK_DRIVETRAIN_LEFT_PWM_CHANNEL);
        this.rightMotor = provider.getVictor(ElectronicsConstants.SIMPLE_TANK_DRIVETRAIN_RIGHT_PWM_CHANNEL);
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        double leftSetpoint = this.driver.getAnalog(AnalogOperation.DriveTrainLeft);
        double rightSetpoint = this.driver.getAnalog(AnalogOperation.DriveTrainRight);

        this.logger.logNumber(LoggingKey.DriveTrainLeftVelocityGoal, leftSetpoint);
        this.logger.logNumber(LoggingKey.DriveTrainRightVelocityGoal, rightSetpoint);

        if (TuningConstants.TANK_DRIVETRAIN_INVERT_LEFT)
        {
            leftSetpoint *= -1.0;
        }

        if (TuningConstants.TANK_DRIVETRAIN_INVERT_RIGHT)
        {
            rightSetpoint *= -1.0;
        }

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
        this.leftMotor.set(0.0);
        this.rightMotor.set(0.0);
    }
}
