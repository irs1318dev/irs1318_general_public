package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.mechanisms.*;
import frc.lib.robotprovider.*;
import frc.robot.ElectronicsConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.*;

import com.google.inject.Inject;

public class OneMotorSparkMechanism implements IMechanism
{
    private static final int slotId = 1;

    private final IDriver driver;
    private final ILogger logger;
    private final ISparkMax motor;
    private final SmartDashboardSelectionManager selectionManager;

    private double velocity;
    private double position;
    private boolean reverseLimitSwtichStatus;
    private boolean forwardLimitSwitchStatus;

    private SmartDashboardSelectionManager.MotorMode currentMode;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private int kAccel;
    private int kCruiseVel;
    private double kMinOutput;
    private double kMaxOutput;
    private boolean useBrakeMode;
    private boolean invertOutput;

    @Inject
    public OneMotorSparkMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        SmartDashboardSelectionManager selectionManager)
    {
        this.driver = driver;
        this.logger = logger;
        this.selectionManager = selectionManager;
        this.motor = provider.getSparkMax(ElectronicsConstants.ONEMOTOR_PRIMARY_MOTOR_CHANNEL, SparkMaxMotorType.Brushless);
        this.motor.setRelativeEncoder();
        this.motor.setNeutralMode(MotorNeutralMode.Brake);
        this.motor.setInvertOutput(TuningConstants.ONEMOTOR_INVERT_OUTPUT);
        this.motor.reset();

        this.motor.setControlMode(SparkMaxControlMode.PercentOutput);
        this.motor.setPIDFSmartMotion(
            TuningConstants.ONEMOTOR_PID_KP,
            TuningConstants.ONEMOTOR_PID_KI,
            TuningConstants.ONEMOTOR_PID_KD,
            TuningConstants.ONEMOTOR_PID_KF,
            0,
            TuningConstants.ONEMOTOR_PID_MM_CRUISE_VELOC,
            TuningConstants.ONEMOTOR_PID_MM_ACCEL,
            TuningConstants.ONEMOTOR_PID_MIN_OUTPUT,
            TuningConstants.ONEMOTOR_PID_MAX_OUTPUT,
            OneMotorSparkMechanism.slotId);

        this.currentMode = SmartDashboardSelectionManager.MotorMode.PercentOutput;
        this.kP = TuningConstants.ONEMOTOR_PID_KP;
        this.kI = TuningConstants.ONEMOTOR_PID_KI;
        this.kD = TuningConstants.ONEMOTOR_PID_KD;
        this.kF = TuningConstants.ONEMOTOR_PID_KF;
        this.kAccel = TuningConstants.ONEMOTOR_PID_MM_ACCEL;
        this.kCruiseVel = TuningConstants.ONEMOTOR_PID_MM_CRUISE_VELOC;
        this.kMinOutput = TuningConstants.ONEMOTOR_PID_MIN_OUTPUT;
        this.kMaxOutput = TuningConstants.ONEMOTOR_PID_MAX_OUTPUT;
        this.useBrakeMode = true;
        this.invertOutput = TuningConstants.ONEMOTOR_INVERT_OUTPUT;

        if (TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED || TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED)
        {
            this.motor.setForwardLimitSwitch(
                TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED,
                TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
            this.motor.setReverseLimitSwitch(
                TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED,
                TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);
        }

        this.velocity = 0.0;
        this.position = 0.0;
    }

    @Override
    public void readSensors()
    {
        this.velocity = this.motor.getVelocity();
        this.position = this.motor.getPosition();

        if (TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED || TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED)
        {
            this.forwardLimitSwitchStatus = this.motor.getForwardLimitSwitchStatus();
            this.reverseLimitSwtichStatus = this.motor.getReverseLimitSwitchStatus();
        }

        this.logger.logNumber(LoggingKey.OneMotorSparkVelocity, this.velocity);
        this.logger.logNumber(LoggingKey.OneMotorSparkPosition, this.position);
        this.logger.logBoolean(LoggingKey.OneMotorSparkReverseLimit, this.reverseLimitSwtichStatus);
        this.logger.logBoolean(LoggingKey.OneMotorSparkForwardLimit, this.forwardLimitSwitchStatus);
    }

    @Override
    public void update()
    {
        boolean shouldUpdatePID = false;
        double newKP = this.selectionManager.getSelectedKP();
        if (newKP != this.kP)
        {
            shouldUpdatePID = true;
        }

        double newKI = this.selectionManager.getSelectedKI();
        if (newKI != this.kI)
        {
            shouldUpdatePID = true;
        }

        double newKD = this.selectionManager.getSelectedKD();
        if (newKD != this.kD)
        {
            shouldUpdatePID = true;
        }

        double newKF = this.selectionManager.getSelectedKF();
        if (newKF != this.kF)
        {
            shouldUpdatePID = true;
        }

        int newCruiseVel = (int)this.selectionManager.getSelectedCruiseVelocity();
        if (newCruiseVel != this.kCruiseVel)
        {
            shouldUpdatePID = true;
        }

        int newAccel = (int)this.selectionManager.getSelectedAcceleration();
        if (newAccel != this.kAccel)
        {
            shouldUpdatePID = true;
        }

        double newMinOutput = this.selectionManager.getSelectedMinOutput();
        if (newMinOutput != this.kMinOutput)
        {
            shouldUpdatePID = true;
        }

        double newMaxOutput = this.selectionManager.getSelectedMaxOutput();
        if (newMaxOutput != this.kMaxOutput)
        {
            shouldUpdatePID = true;
        }

        if (shouldUpdatePID)
        {
            this.kP = newKP;
            this.kI = newKI;
            this.kD = newKD;
            this.kF = newKF;
            this.kAccel = newAccel;
            this.kCruiseVel = newCruiseVel;
            this.kMinOutput = newMinOutput;
            this.kMaxOutput = newMaxOutput;

            this.motor.setPIDFSmartMotion(
                this.kP,
                this.kI,
                this.kD,
                this.kF,
                0,
                (int)this.kCruiseVel,
                (int)this.kAccel,
                this.kMinOutput,
                this.kMaxOutput,
                OneMotorSparkMechanism.slotId);
        }

        boolean newInvertOutput = this.selectionManager.getSelectedInvertOutput();
        if (newInvertOutput != this.invertOutput)
        {
            this.invertOutput = newInvertOutput;
            this.motor.setInvertOutput(TuningConstants.ONEMOTOR_INVERT_OUTPUT);
        }

        boolean newBrakeMode = this.selectionManager.getSelectedBrakeMode();
        if (newBrakeMode != this.useBrakeMode)
        {
            this.useBrakeMode = newBrakeMode;
            this.motor.setNeutralMode(this.useBrakeMode ? MotorNeutralMode.Brake : MotorNeutralMode.Coast);
        }

        SmartDashboardSelectionManager.MotorMode newMode = this.selectionManager.getSelectedMotorMode();
        if (newMode != this.currentMode)
        {
            this.currentMode = newMode;
            switch (this.currentMode)
            {
                case PercentOutput:
                    this.motor.setControlMode(SparkMaxControlMode.PercentOutput);
                    break;

                case PositionPID:
                    this.motor.setControlMode(SparkMaxControlMode.Position);
                    break;

                case VelocityPID:
                    this.motor.setControlMode(SparkMaxControlMode.Velocity);
                    break;

                case TrapezoidalMotionProfile:
                    this.motor.setControlMode(SparkMaxControlMode.SmartMotionPosition);
                    break;
            }
        }

        double setpoint = this.driver.getAnalog(AnalogOperation.OneMotorPower);

        double maxSetpointValue = 1.0;
        switch (this.currentMode)
        {
            case PositionPID:
            case TrapezoidalMotionProfile:
                maxSetpointValue = TuningConstants.ONEMOTOR_PID_MAX_POSITION;
                break;

            case VelocityPID:
                maxSetpointValue = TuningConstants.ONEMOTOR_PID_MAX_VELOCITY;
                break;
        }

        setpoint *= maxSetpointValue;

        this.logger.logNumber(LoggingKey.OneMotorSparkSetpoint, setpoint);
        this.motor.set(setpoint);

        this.logger.logNumber(LoggingKey.OneMotorSparkOutput, this.motor.getOutput());
    }

    @Override
    public void stop()
    {
        this.motor.reset();
        this.motor.stop();
    }
}
