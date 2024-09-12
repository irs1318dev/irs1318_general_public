package frc.robot.mechanisms;

import javax.inject.Inject;
import javax.inject.Singleton;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IMotor;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.RobotMode;
import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;

@Singleton
public class StingerMechanism implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;

    private final IMotor motor;

    @Inject
    public StingerMechanism(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        this.motor = provider.getTalon(ElectronicsConstants.STINGER_MOTOR_CHANNEL);
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update(RobotMode mode)
    {
        // note: motor is inverted
        if (this.driver.getDigital(DigitalOperation.StingerOut))
        {
            this.motor.set(-TuningConstants.STINGER_MAX_VELOCTIY);
        }
        else if (this.driver.getDigital(DigitalOperation.StingerIn))
        {
            this.motor.set(TuningConstants.STINGER_MAX_VELOCTIY);
        }
        else
        {
            this.motor.set(-TuningConstants.STINGER_SLOW_BACK_VELOCTIY);
        }
    }

    @Override
    public void stop()
    {
        this.motor.set(0.0);
    }
}
