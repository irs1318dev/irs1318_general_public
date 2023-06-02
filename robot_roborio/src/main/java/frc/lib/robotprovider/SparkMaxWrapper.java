package frc.lib.robotprovider;

import com.revrobotics.*;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxWrapper implements ISparkMax
{
    final CANSparkMax wrappedObject;
    private SparkMaxPIDController pidController;
    private RelativeEncoder wrappedEncoder;
    private SparkMaxLimitSwitch wrappedFwdLimitSwitch;
    private SparkMaxLimitSwitch wrappedRevLimitSwitch;

    private SparkMaxControlMode currentMode;
    private int selectedSlot;

    public SparkMaxWrapper(int deviceID, SparkMaxMotorType motorType)
    {
        MotorType type = MotorType.kBrushless;
        switch (motorType)
        {
            case Brushed:
                type = MotorType.kBrushed;
                break;

            case Brushless:
                type = MotorType.kBrushless;
                break;
        }

        this.wrappedObject = new CANSparkMax(deviceID, type);
        this.currentMode = SparkMaxControlMode.PercentOutput;
    }

    public void setControlMode(SparkMaxControlMode mode)
    {
        this.currentMode = mode;
    }

    public void set(double value)
    {
        if (this.currentMode == SparkMaxControlMode.PercentOutput)
        {
            this.wrappedObject.set(value);
            return;
        }

        if (this.pidController == null)
        {
            this.pidController = this.wrappedObject.getPIDController();
        }

        CANSparkMax.ControlType controlType;
        switch (this.currentMode)
        {
            case Position:
                controlType = CANSparkMax.ControlType.kPosition;
                break;

            case Velocity:
                controlType = CANSparkMax.ControlType.kVelocity;
                break;

            case Voltage:
                controlType = CANSparkMax.ControlType.kVoltage;
                break;

            default:
            case PercentOutput:
                throw new RuntimeException("unexpected control mode " + this.currentMode);
        }

        RevErrorCodeHelper.printError(
            this.pidController.setReference(value, controlType, this.selectedSlot),
            "SparkMaxWrapper.set");
    }

    public void follow(ISparkMax sparkMax)
    {
        RevErrorCodeHelper.printError(
            this.wrappedObject.follow(((SparkMaxWrapper)sparkMax).wrappedObject),
            "SparkMaxWrapper.follow");
    }

    public void setFeedbackFramePeriod(SparkMaxPeriodicFrameType frameType, int periodMS)
    {
        PeriodicFrame type = PeriodicFrame.kStatus0;
        switch (frameType)
        {
            case Status0:
                type = PeriodicFrame.kStatus0;
                break;
            case Status1:
                type = PeriodicFrame.kStatus1;
                break;
            case Status2:
                type = PeriodicFrame.kStatus2;
                break;
        }

        RevErrorCodeHelper.printError(
            this.wrappedObject.setPeriodicFramePeriod(type, periodMS),
            "SparkMaxWrapper.setFeedbackFramePeriod");
    }

    public void setVelocityMeasurements(int periodMS, int windowSize)
    {
        RevErrorCodeHelper.printError(
            this.wrappedEncoder.setAverageDepth(windowSize),
            "SparkMaxWrapper.setVelocityMeasurements-setAverageDepth");
        RevErrorCodeHelper.printError(
            this.wrappedEncoder.setMeasurementPeriod(periodMS),
            "SparkMaxWrapper.setVelocityMeasurements-setMeasurementPeriod");
    }

    public void setSelectedSlot(int slotId)
    {
        this.selectedSlot = slotId;
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        if (this.pidController == null)
        {
            this.pidController = this.wrappedObject.getPIDController();
        }

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDF-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDF-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDF-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDF-f");
    }

    public void setPIDF(double p, double i, double d, double f, double minOutput, double maxOutput, int slotId)
    {
        if (this.pidController == null)
        {
            this.pidController = this.wrappedObject.getPIDController();
        }

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDF-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDF-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDF-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDF-f");
        RevErrorCodeHelper.printError(
            this.pidController.setOutputRange(minOutput, maxOutput),
            "SparkMaxWrapper.setPIDF-output");
    }

    public void setPIDF(double p, double i, double d, double f, int izone, int slotId)
    {
        if (this.pidController == null)
        {
            this.pidController = this.wrappedObject.getPIDController();
        }

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDF-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDF-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDF-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDF-f");
        RevErrorCodeHelper.printError(
            this.pidController.setIZone(izone, slotId),
            "SparkMaxWrapper.setPIDF-izone");
    }

    public void setPIDF(double p, double i, double d, double f, int izone, double minOutput, double maxOutput, int slotId)
    {
        if (this.pidController == null)
        {
            this.pidController = this.wrappedObject.getPIDController();
        }

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDF-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDF-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDF-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDF-f");
        RevErrorCodeHelper.printError(
            this.pidController.setIZone(izone, slotId),
            "SparkMaxWrapper.setPIDF-izone");
        RevErrorCodeHelper.printError(
            this.pidController.setOutputRange(minOutput, maxOutput),
            "SparkMaxWrapper.setPIDF-output");
    }

    public void setPIDFSmartMotion(double p, double i, double d, double f, int izone, int velocity, int acceleration, int slotId)
    {
        if (this.pidController == null)
        {
            this.pidController = this.wrappedObject.getPIDController();
        }

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-f");
        RevErrorCodeHelper.printError(
            this.pidController.setIZone(izone, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-izone");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionMaxVelocity(velocity, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-maxvelocity");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionMaxAccel(acceleration, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-maxaccel");
    }

    public void setPIDFSmartMotion(double p, double i, double d, double f, int izone, int velocity, int acceleration, double minOutput, double maxOutput, int slotId)
    {
        if (this.pidController == null)
        {
            this.pidController = this.wrappedObject.getPIDController();
        }

        RevErrorCodeHelper.printError(
            this.pidController.setP(p, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-p");
        RevErrorCodeHelper.printError(
            this.pidController.setI(i, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-i");
        RevErrorCodeHelper.printError(
            this.pidController.setD(d, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-d");
        RevErrorCodeHelper.printError(
            this.pidController.setFF(f, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-f");
        RevErrorCodeHelper.printError(
            this.pidController.setIZone(izone, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-izone");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionMaxVelocity(velocity, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-maxvelocity");
        RevErrorCodeHelper.printError(
            this.pidController.setSmartMotionMaxAccel(acceleration, slotId),
            "SparkMaxWrapper.setPIDFSmartMotion-maxaccel");
        RevErrorCodeHelper.printError(
            this.pidController.setOutputRange(minOutput, maxOutput),
            "SparkMaxWrapper.setPIDFSmartMotion-output");
    }

    public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        SparkMaxLimitSwitch.Type polarity = SparkMaxLimitSwitch.Type.kNormallyClosed;
        if (normallyOpen)
        {
            polarity = SparkMaxLimitSwitch.Type.kNormallyOpen;
        }

        this.wrappedFwdLimitSwitch = this.wrappedObject.getForwardLimitSwitch(polarity);
        RevErrorCodeHelper.printError(
            this.wrappedFwdLimitSwitch.enableLimitSwitch(enabled),
            "SparkMaxWrapper.setForwardLimitSwitch");
    }

    public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        SparkMaxLimitSwitch.Type polarity = SparkMaxLimitSwitch.Type.kNormallyClosed;
        if (normallyOpen)
        {
            polarity = SparkMaxLimitSwitch.Type.kNormallyOpen;
        }

        this.wrappedRevLimitSwitch = this.wrappedObject.getForwardLimitSwitch(polarity);
        RevErrorCodeHelper.printError(
            this.wrappedRevLimitSwitch.enableLimitSwitch(enabled),
            "SparkMaxWrapper.setReverseLimitSwitch");
    }

    public void setInvertOutput(boolean invert)
    {
        this.wrappedObject.setInverted(invert);
    }

    public void setInvertSensor(boolean invert)
    {
        RevErrorCodeHelper.printError(
            this.wrappedEncoder.setInverted(invert),
            "SparkMaxWrapper.setInvertSensor");
    }

    public void setNeutralMode(MotorNeutralMode neutralMode)
    {
        IdleMode mode;
        if (neutralMode == MotorNeutralMode.Brake)
        {
            mode = IdleMode.kBrake;
        }
        else
        {
            mode = IdleMode.kCoast;
        }

        RevErrorCodeHelper.printError(
            this.wrappedObject.setIdleMode(mode),
            "SparkMaxWrapper.setNeutralMode");
    }

    public void setCurrentLimit(int stallLimit, int freeLimit, int limitRPM)
    {
        RevErrorCodeHelper.printError(
            this.wrappedObject.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM),
            "SparkMaxWrapper.setCurrentLimit");
    }

    public void stop()
    {
        this.wrappedObject.stopMotor();
    }

    public void setPosition(double position)
    {
        if (this.wrappedEncoder == null)
        {
            this.wrappedEncoder = this.wrappedObject.getEncoder();
            if (this.wrappedEncoder == null)
            {
                return;
            }
        }

        RevErrorCodeHelper.printError(
            this.wrappedEncoder.setPosition(position),
            "SparkMaxWrapper.setPosition");
    }

    public void reset()
    {
        this.setPosition(0.0);
    }

    public double getPosition()
    {
        if (this.wrappedEncoder == null)
        {
            this.wrappedEncoder = this.wrappedObject.getEncoder();
            if (wrappedEncoder == null)
            {
                return 0.0;
            }
        }

        return this.wrappedEncoder.getPosition();
    }

    public double getVelocity()
    {
        if (this.wrappedEncoder == null)
        {
            this.wrappedEncoder = this.wrappedObject.getEncoder();
            if (wrappedEncoder == null)
            {
                return 0.0;
            }
        }

        return this.wrappedEncoder.getVelocity();
    }

    public boolean getForwardLimitSwitchStatus()
    {
        if (this.wrappedFwdLimitSwitch == null)
        {
            return false;
        }

        return this.wrappedFwdLimitSwitch.isPressed();
    }

    public boolean getReverseLimitSwitchStatus()
    {
        if (this.wrappedRevLimitSwitch == null)
        {
            return false;
        }

        return this.wrappedRevLimitSwitch.isPressed();
    }
}
