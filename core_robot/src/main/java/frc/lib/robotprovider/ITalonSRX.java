package frc.lib.robotprovider;

public interface ITalonSRX extends ITalonXBase
{
    void follow(ITalonSRX talonSRX);
    void follow(IVictorSPX victorSPX);
    void set(TalonSRXControlMode mode, double value);
    void setControlMode(TalonSRXControlMode mode);
    void setSensorType(TalonSRXFeedbackDevice feedbackDevice);
    void setGeneralFramePeriod(int periodMS);
    void setFeedbackFramePeriod(int periodMS);
    void setPIDFFramePeriod(int periodMS);
    void configureVelocityMeasurements(int periodMS, int windowSize);
    void configureAllowableClosedloopError(int slotId, int error);
    void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId);
    void setInvertSensor(boolean flip);
}
