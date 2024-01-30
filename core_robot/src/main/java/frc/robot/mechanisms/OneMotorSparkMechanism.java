package frc.robot.mechanisms;

import frc.lib.controllers.TrapezoidProfile;
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
    private final ITimer timer;

    private final SmartDashboardSelectionManager selectionManager;
    private SmartDashboardSelectionManager.MotorMode currentMode;
    private double kP;
    private double kI;
    private double kD;
    private double kFV;
    private double kS;
    private int kCruiseVel;
    private int kAccel;
    private int kJerk;
    private double kMinOutput;
    private double kMaxOutput;
    private boolean useBrakeMode;
    private boolean invertOutput;

    private TrapezoidProfile trapezoidProfile;
    private TrapezoidProfile.State curr;
    private TrapezoidProfile.State goal;

    private double velocity;
    private double position;
    private double prevTime;
    public boolean reverseLimitSwtichStatus;
    public boolean forwardLimitSwitchStatus;

    @Inject
    public OneMotorSparkMechanism(
        IDriver driver,
        LoggingManager logger,
        IRobotProvider provider,
        ITimer timer,
        SmartDashboardSelectionManager selectionManager)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;
        this.selectionManager = selectionManager;

        this.motor = provider.getSparkMax(ElectronicsConstants.ONEMOTOR_PRIMARY_MOTOR_CHANNEL, SparkMaxMotorType.Brushless);
        if (TuningConstants.ONEMOTOR_USE_ABSOLUTE)
        {
            this.motor.setAbsoluteEncoder();
        }
        else
        {
            this.motor.setRelativeEncoder();
            this.motor.reset();
        }

        this.currentMode = SmartDashboardSelectionManager.MotorMode.PercentOutput;
        this.kP = TuningConstants.ONEMOTOR_PID_KP;
        this.kI = TuningConstants.ONEMOTOR_PID_KI;
        this.kD = TuningConstants.ONEMOTOR_PID_KD;
        this.kFV = TuningConstants.ONEMOTOR_PID_KFV;
        this.kS = TuningConstants.ONEMOTOR_PID_KS;
        this.kCruiseVel = TuningConstants.ONEMOTOR_PID_MM_CRUISE_VELOC;
        this.kAccel = TuningConstants.ONEMOTOR_PID_MM_ACCEL;
        this.kJerk = TuningConstants.ONEMOTOR_PID_MM_JERK;
        this.kMinOutput = TuningConstants.ONEMOTOR_PID_MIN_OUTPUT;
        this.kMaxOutput = TuningConstants.ONEMOTOR_PID_MAX_OUTPUT;
        this.useBrakeMode = true;
        this.invertOutput = TuningConstants.ONEMOTOR_INVERT_OUTPUT;

        this.motor.setPositionConversionFactor(1.0);
        this.motor.setVelocityConversionFactor(1.0);
        this.motor.setNeutralMode(MotorNeutralMode.Brake);
        this.motor.setInvertOutput(TuningConstants.ONEMOTOR_INVERT_OUTPUT);
        this.motor.burnFlash();

        if (TuningConstants.ONEMOTOR_HAS_FOLLOWER)
        {
            ISparkMax follower = provider.getSparkMax(ElectronicsConstants.ONEMOTOR_FOLLOWER_MOTOR_CHANNEL, SparkMaxMotorType.Brushless);
            follower.setNeutralMode(MotorNeutralMode.Brake);
            follower.follow(this.motor);
        }

        this.motor.setForwardLimitSwitch(
            TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.motor.setReverseLimitSwitch(
            TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);

        if (TuningConstants.ONEMOTOR_FORWARD_LIMIT_SWITCH_ENABLED || TuningConstants.ONEMOTOR_REVERSE_LIMIT_SWITCH_ENABLED)
        {
            if (TuningConstants.ONEMOTOR_PID_POSITIONAL)
            {
                if (TuningConstants.ONEMOTOR_PID_POSITIONAL_MM)
                {
                    this.motor.setControlMode(SparkMaxControlMode.Position);

                    this.trapezoidProfile = new TrapezoidProfile(TuningConstants.ONEMOTOR_PID_MM_CRUISE_VELOC, TuningConstants.ONEMOTOR_PID_MM_ACCEL);
                    this.curr = new TrapezoidProfile.State(0.0, 0.0);
                    this.goal = new TrapezoidProfile.State(0.0, 0.0);

                    this.motor.setPIDF(
                        TuningConstants.ONEMOTOR_PID_KP,
                        TuningConstants.ONEMOTOR_PID_KI,
                        TuningConstants.ONEMOTOR_PID_KD,
                        TuningConstants.ONEMOTOR_PID_KFV,
                        TuningConstants.ONEMOTOR_PID_MIN_OUTPUT,
                        TuningConstants.ONEMOTOR_PID_MAX_OUTPUT,
                        OneMotorSparkMechanism.slotId);

                    // this.motor.setPIDFSmartMotion(
                    //     TuningConstants.ONEMOTOR_PID_KP,
                    //     TuningConstants.ONEMOTOR_PID_KI,
                    //     TuningConstants.ONEMOTOR_PID_KD,
                    //     TuningConstants.ONEMOTOR_PID_KFV,
                    //     0,
                    //     TuningConstants.ONEMOTOR_PID_MM_CRUISE_VELOC,
                    //     TuningConstants.ONEMOTOR_PID_MM_ACCEL,
                    //     TuningConstants.ONEMOTOR_PID_MIN_OUTPUT,
                    //     TuningConstants.ONEMOTOR_PID_MAX_OUTPUT,
                    //     OneMotorSparkMechanism.slotId);
                }
                else
                {
                    this.motor.setControlMode(SparkMaxControlMode.Position);

                    this.motor.setPIDF(
                        TuningConstants.ONEMOTOR_PID_KP,
                        TuningConstants.ONEMOTOR_PID_KI,
                        TuningConstants.ONEMOTOR_PID_KD,
                        TuningConstants.ONEMOTOR_PID_KFV,
                        TuningConstants.ONEMOTOR_PID_MIN_OUTPUT,
                        TuningConstants.ONEMOTOR_PID_MAX_OUTPUT,
                        OneMotorSparkMechanism.slotId);
                }
            }
            else
            {
                this.motor.setControlMode(SparkMaxControlMode.Velocity);

                this.motor.setPIDF(
                    TuningConstants.ONEMOTOR_PID_KP,
                    TuningConstants.ONEMOTOR_PID_KI,
                    TuningConstants.ONEMOTOR_PID_KD,
                    TuningConstants.ONEMOTOR_PID_KFV,
                    TuningConstants.ONEMOTOR_PID_MIN_OUTPUT,
                    TuningConstants.ONEMOTOR_PID_MAX_OUTPUT,
                    OneMotorSparkMechanism.slotId);
            }
        }
        else
        {
            this.motor.setControlMode(SparkMaxControlMode.PercentOutput);
        }

        this.velocity = 0.0;
        this.position = 0.0;

        this.prevTime = 0.0;

        this.reverseLimitSwtichStatus = false;
        this.forwardLimitSwitchStatus = false;
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
        double currTime = this.timer.get();
        double deltaT = currTime - this.prevTime;

        double setpoint = this.driver.getAnalog(AnalogOperation.OneMotorPower);

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

        double newKFV = this.selectionManager.getSelectedKFV();
        if (newKFV != this.kFV)
        {
            shouldUpdatePID = true;
        }

        double newKS = this.selectionManager.getSelectedKS();
        if (newKS != this.kS)
        {
            shouldUpdatePID = true;
        }

        int newCruiseVel = (int)this.selectionManager.getSelectedCruiseVelocity();
        if (newCruiseVel != this.kCruiseVel)
        {
            shouldUpdatePID = true;
        }

        int newAccel = TuningConstants.ONEMOTOR_PID_MM_ACCEL; // (int)this.selectionManager.getSelectedAcceleration();
        if (newAccel != this.kAccel)
        {
            shouldUpdatePID = true;
        }

        int newJerk = TuningConstants.ONEMOTOR_PID_MM_JERK; // (int)this.selectionManager.getSelectedJerk();
        if (newJerk != this.kJerk)
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
            this.kFV = newKFV;
            this.kS = newKS;
            this.kCruiseVel = newCruiseVel;
            this.kAccel = newAccel;
            this.kJerk = newJerk;
            this.kMinOutput = newMinOutput;
            this.kMaxOutput = newMaxOutput;

            this.motor.setPIDF(
                this.kP,
                this.kI,
                this.kD,
                this.kFV,
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
        if (TuningConstants.ONEMOTOR_PID_POSITIONAL_MM)
        {
            if (this.goal.updatePosition(setpoint))
            {
                this.curr.updatePosition(this.position);
            }

            this.trapezoidProfile.update(deltaT, this.curr, this.goal);
            setpoint = this.curr.getPosition();
        }

        this.motor.set(setpoint);

        this.logger.logNumber(LoggingKey.OneMotorSparkOutput, this.motor.getOutput());
        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        if (TuningConstants.ONEMOTOR_USE_ABSOLUTE)
        {
            this.motor.reset();
        }

        this.motor.stop();
    }
}
