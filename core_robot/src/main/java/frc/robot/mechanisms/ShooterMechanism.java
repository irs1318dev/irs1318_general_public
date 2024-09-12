package frc.robot.mechanisms;

import javax.inject.Inject;
import javax.inject.Singleton;

import frc.lib.controllers.PIDHandler;
import frc.lib.driver.IDriver;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.DoubleSolenoidValue;
import frc.lib.robotprovider.IDoubleSolenoid;
import frc.lib.robotprovider.IEncoder;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IMotor;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ISolenoid;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.PneumaticsModuleType;
import frc.lib.robotprovider.RobotMode;
import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

@Singleton
public class ShooterMechanism implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;

    private final PowerManager powerManager;

    private final IDoubleSolenoid kicker;
    private final IDoubleSolenoid hood;
    private final IMotor talon;
    private final IEncoder encoder;
    private final ISolenoid readyLight;

    private final PIDHandler pid;

    private int encoderTicks;
    private double encoderRate;

    @Inject
    public ShooterMechanism(IDriver driver, ITimer timer, PowerManager powerManager, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        this.powerManager = powerManager;

        this.kicker = provider.getDoubleSolenoid(ElectronicsConstants.PCM_B_MODULE, PneumaticsModuleType.PneumaticsControlModule, ElectronicsConstants.SHOOTER_KICKER_CHANNEL_A, ElectronicsConstants.SHOOTER_KICKER_CHANNEL_B);
        this.hood = provider.getDoubleSolenoid(ElectronicsConstants.PCM_A_MODULE, PneumaticsModuleType.PneumaticsControlModule, ElectronicsConstants.SHOOTER_HOOD_CHANNEL_A, ElectronicsConstants.SHOOTER_HOOD_CHANNEL_B);
        this.talon = provider.getTalon(ElectronicsConstants.SHOOTER_TALON_CHANNEL);
        this.encoder = provider.getEncoder(ElectronicsConstants.SHOOTER_ENCODER_CHANNEL_A, ElectronicsConstants.SHOOTER_ENCODER_CHANNEL_B);
        this.readyLight = provider.getSolenoid(ElectronicsConstants.PCM_B_MODULE, PneumaticsModuleType.PneumaticsControlModule, ElectronicsConstants.SHOOTER_READY_LIGHT_PORT);

        this.pid = new PIDHandler(
            TuningConstants.SHOOTER_VELOCITY_PID_KP_DEFAULT,
            TuningConstants.SHOOTER_VELOCITY_PID_KI_DEFAULT,
            TuningConstants.SHOOTER_VELOCITY_PID_KD_DEFAULT,
            TuningConstants.SHOOTER_VELOCITY_PID_KF_DEFAULT,
            TuningConstants.SHOOTER_VELOCITY_PID_KS_DEFAULT,
            -TuningConstants.SHOOTER_MAX_POWER_LEVEL, 
            TuningConstants.SHOOTER_MAX_POWER_LEVEL,
            timer);

        this.encoderTicks = 0;
        this.encoderRate = 0.0;
    }

    @Override
    public void readSensors()
    {
        this.encoderTicks = this.encoder.get();
        this.encoderRate = this.encoder.getRate();

        this.logger.logNumber(LoggingKey.ShooterRate, this.encoderRate);
        this.logger.logNumber(LoggingKey.ShooterTicks, this.encoderTicks);
    }

    @Override
    public void update(RobotMode mode)
    {
        boolean spin = this.driver.getDigital(DigitalOperation.ShooterSpin);

        // The velocity set in the analog operation
        double velocityGoal = this.driver.getAnalog(AnalogOperation.ShooterSpeed);
        this.logger.logNumber(LoggingKey.ShooterGoal, velocityGoal);

        double power = 0.0;
        boolean shouldLight = false;
        if (spin)
        {
            double speedPercentage = this.encoderRate / TuningConstants.SHOOTER_MAX_COUNTER_RATE;
            shouldLight = velocityGoal != 0.0 && Helpers.WithinDelta(speedPercentage, velocityGoal, TuningConstants.SHOOTER_DEVIANCE);

            // Calculate the power required to reach the velocity goal     
            power = this.pid.calculateVelocity(velocityGoal, this.encoderRate);

            if (TuningConstants.SHOOTER_SCALE_BASED_ON_VOLTAGE)
            {
                power *= (TuningConstants.SHOOTER_VELOCITY_TUNING_VOLTAGE / this.powerManager.getBatteryVoltage());
                power = Helpers.EnforceRange(power, -TuningConstants.SHOOTER_MAX_POWER_LEVEL, TuningConstants.SHOOTER_MAX_POWER_LEVEL);
            }
        }

        this.readyLight.set(shouldLight);

        // Set the motor power with the calculated value
        this.talon.set(power);
        this.logger.logNumber(LoggingKey.ShooterPower, power);

        // lower the kicker whenever we are rotating in or out, or when we are performing a shot macro
        if (this.driver.getDigital(DigitalOperation.ShooterLowerKicker))
        {
            this.kicker.set(DoubleSolenoidValue.Reverse);
        }
        else
        {
            this.kicker.set(DoubleSolenoidValue.Forward);
        }

        if (this.driver.getDigital(DigitalOperation.ShooterExtendHood))
        {
            this.hood.set(DoubleSolenoidValue.Forward);
        }
        else
        {
            this.hood.set(DoubleSolenoidValue.Reverse);
        }
    }

    @Override
    public void stop()
    {
        this.kicker.set(DoubleSolenoidValue.Off);
        this.hood.set(DoubleSolenoidValue.Off);
        this.talon.set(0.0);
        this.readyLight.set(false);
    }

    public double getRate()
    {
        return this.encoderRate;
    }
}
