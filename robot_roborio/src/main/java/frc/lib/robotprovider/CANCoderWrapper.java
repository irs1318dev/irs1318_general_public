package frc.lib.robotprovider;

import com.ctre.phoenix.sensors.*;

public class CANCoderWrapper implements ICANCoder
{
    private final CANCoder wrappedObject;

    public CANCoderWrapper(int deviceNumber)
    {
        this.wrappedObject = new CANCoder(deviceNumber);
    }

    public CANCoderWrapper(int deviceNumber, String canbus)
    {
        this.wrappedObject = new CANCoder(deviceNumber, canbus);
    }

    public double getPosition()
    {
        return this.wrappedObject.getPosition();
    }

    public double getVelocity()
    {
        return this.wrappedObject.getVelocity();
    }

    public double getAbsolutePosition()
    {
        return this.wrappedObject.getAbsolutePosition();
    }

    public void setPosition(double newPosition)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setPosition(newPosition),
            "CANCoder.setPosition");
    }

    public void configSensorDirection(boolean clockwisePositive)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configSensorDirection(clockwisePositive),
            "CANCoder.configSensorDirection");
    }

    public void configAbsoluteRange(boolean useZeroToThreeSixty)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configAbsoluteSensorRange(useZeroToThreeSixty ? AbsoluteSensorRange.Unsigned_0_to_360 : AbsoluteSensorRange.Signed_PlusMinus180),
            "CANCoder.configAbsoluteRange");
    }

    public void configMagnetOffset(double offsetDegrees)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.configMagnetOffset(offsetDegrees),
            "CANCoder.configMagnetOffset");
    }
}