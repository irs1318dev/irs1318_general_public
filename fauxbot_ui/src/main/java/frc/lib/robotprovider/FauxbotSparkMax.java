package frc.lib.robotprovider;

import frc.lib.controllers.PIDHandler;

public class FauxbotSparkMax extends FauxbotAdvancedMotorBase implements ISparkMax
{
    private FauxbotEncoder innerEncoder;
    private PIDHandler pidHandler;

    private SparkMaxControlMode currentMode;
    private double kp;
    private double ki;
    private double kd;
    private double kf;

    public FauxbotSparkMax(int deviceID, SparkMaxMotorType motorType)
    {
        super(deviceID);

        this.currentMode = SparkMaxControlMode.PercentOutput;
        this.innerEncoder = new FauxbotEncoder(new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, this.getClass(), this.connection.getPort()));
    }

    public void setControlMode(SparkMaxControlMode mode)
    {
        this.currentMode = mode;
        this.resetPID();
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
        this.resetPID();
    }

    public void setPIDF(double p, double i, double d, double f, double minOutput, double maxOutput, int slotId)
    {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
        this.resetPID();
    }

    public void setPIDF(double p, double i, double d, double f, int izone, int slotId)
    {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
        this.resetPID();
    }

    public void setPIDF(double p, double i, double d, double f, int izone, double minOutput, double maxOutput, int slotId)
    {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
        this.resetPID();
    }

    public void setPIDFSmartMotion(double p, double i, double d, double f, int izone, int velocity, int acceleration, int slotId)
    {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
        this.resetPID();
    }

    public void setPIDFSmartMotion(double p, double i, double d, double f, int izone, int velocity, int acceleration, double minOutput, double maxOutput, int slotId)
    {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;
        this.resetPID();
    }

    public void set(double newValue)
    {
        if (this.currentMode == SparkMaxControlMode.Velocity)
        {
            super.set(this.pidHandler.calculateVelocity(newValue, innerEncoder.getRate()));
        }
        else if (this.currentMode == SparkMaxControlMode.Position)
        {
            super.set(this.pidHandler.calculatePosition(newValue, innerEncoder.get()));
        }
        else
        {
            super.set(newValue);
        }
    }

    public double getPosition()
    {
        return (double)this.innerEncoder.getDistance();
    }

    public void follow(ISparkMax sparkMax)
    {
    }

    public void setAbsoluteEncoder()
    {
    }

    public void setRelativeEncoder()
    {
    }

    public void setRelativeEncoder(SparkMaxRelativeEncoderType encoderType, int resolution)
    {
    }

    public void setFeedbackFramePeriod(SparkMaxPeriodicFrameType frameType, int periodMS)
    {
    }

    public void setEncoderAverageDepth(int windowSize)
    {
    }

    public void setVelocityMeasurementPeriod(int periodMS)
    {
    }

    public void setSelectedSlot(int slotId)
    {
    }

    public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
    {
    }

    public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
    {
    }

    public void setInvertOutput(boolean invert)
    {
    }

    public void setInvertSensor(boolean invert)
    {
    }

    public void setNeutralMode(MotorNeutralMode neutralMode)
    {
    }

    public void setCurrentLimit(int stallLimit, int freeLimit, int limitRPM)
    {
    }

    public void stop()
    {
    }

    public void setAbsoluteOffset(double zeroOffset)
    {
    }

    public void setPosition(double position)
    {
    }

    public void reset()
    {
    }

    public void burnFlash()
    {
    }

    public void setPositionConversionFactor(double ratio)
    {
    }

    public void setVelocityConversionFactor(double ratio)
    {
    }

    public void setPositionPIDWrappingSettings(boolean enable, double minInput, double maxInput)
    {
    }

    public double getVelocity()
    {
        return 0.0;
    }

    public boolean getForwardLimitSwitchStatus()
    {
        return false;
    }

    public boolean getReverseLimitSwitchStatus()
    {
        return false;
    }

    private void resetPID()
    {
        if (this.currentMode == SparkMaxControlMode.Position ||
            this.currentMode == SparkMaxControlMode.Velocity)
        {
            ITimer timer = new FauxbotTimer();
            timer.start();
            this.pidHandler = new PIDHandler(this.kp, this.ki, this.kd, this.kf, 1.0, -4096.0, 4096.0, timer);
        }
        else
        {
            this.pidHandler = null;
        }
    }
}
