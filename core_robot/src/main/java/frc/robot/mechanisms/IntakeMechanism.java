package frc.robot.mechanisms;

import javax.inject.Inject;
import javax.inject.Singleton;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.DoubleSolenoidValue;
import frc.lib.robotprovider.IAnalogInput;
import frc.lib.robotprovider.IDoubleSolenoid;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IMotor;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ISolenoid;
import frc.lib.robotprovider.PneumaticsModuleType;
import frc.lib.robotprovider.RobotMode;
import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;

@Singleton
public class IntakeMechanism implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;

    private final IMotor motor;
    private final IDoubleSolenoid intakeSolenoid;
    private final ISolenoid intakeLight;
    private final IAnalogInput throughBeamSensor;

    private boolean isThroughBeamBroken;

    @Inject
    public IntakeMechanism(IDriver driver, LoggingManager logger, IRobotProvider provider) 
    {
        this.driver = driver;
        this.logger = logger;

        this.motor = provider.getTalon(ElectronicsConstants.INTAKE_MOTOR_CHANNEL);
        this.intakeSolenoid = provider.getDoubleSolenoid(
            ElectronicsConstants.PCM_A_MODULE,
            PneumaticsModuleType.PneumaticsControlModule,
            ElectronicsConstants.INTAKE_SOLENOID_CHANNEL_A,
            ElectronicsConstants.INTAKE_SOLENOID_CHANNEL_B);
        this.intakeLight = provider.getSolenoid(
            ElectronicsConstants.PCM_B_MODULE,
            PneumaticsModuleType.PneumaticsControlModule,
            ElectronicsConstants.INTAKE_LIGHT_CHANNEL);
        this.throughBeamSensor = provider.getAnalogInput(ElectronicsConstants.INTAKE_THROUGH_BEAM_SENSOR_CHANNEL);

        this.isThroughBeamBroken = false;
    }

    @Override
    public void readSensors()
    {
        this.isThroughBeamBroken = this.throughBeamSensor.getVoltage() < 2.5;

        this.logger.logBoolean(LoggingKey.IntakeThroughBeam, this.isThroughBeamBroken);
    }

    @Override
    public void update(RobotMode mode)
    {
        // Check for "intake base" extend desire, and extend or retract appropriately
        if (this.driver.getDigital(DigitalOperation.IntakeExtend))
        {
            this.intakeSolenoid.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeRetract))
        {
            this.intakeSolenoid.set(DoubleSolenoidValue.Reverse);
        }

        // Roll the intake in, out, or not at all when appropriate
        if (this.driver.getDigital(DigitalOperation.IntakeRotatingIn))
        {
            this.motor.set(TuningConstants.INTAKE_IN_POWER_LEVEL);
        }
        else if (this.driver.getDigital(DigitalOperation.IntakeRotatingOut))
        {
            this.motor.set(TuningConstants.INTAKE_OUT_POWER_LEVEL);
        }
        else 
        {
            this.motor.set(0.0);
        }

        // Turn on the intake light if the through beam sensor is broken
        this.intakeLight.set(this.isThroughBeamBroken);
    }

    @Override
    public void stop()
    {
        this.motor.set(0.0);
        this.intakeSolenoid.set(DoubleSolenoidValue.Off);
        this.intakeLight.set(false);
    }
}