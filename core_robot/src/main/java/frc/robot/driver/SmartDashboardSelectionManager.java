package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.robotprovider.*;

@Singleton
public class SmartDashboardSelectionManager
{
    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;

    private final ISendableChooser<MotorMode> motorModeChooser;
    private final IDoubleSubscriber kpSlider;
    private final IDoubleSubscriber kiSlider;
    private final IDoubleSubscriber kdSlider;
    private final IDoubleSubscriber kfSlider;
    private final IDoubleSubscriber kAccelSlider;
    private final IDoubleSubscriber kCruiseVelSlider;
    private final IBooleanSubscriber invertOutputCheckbox;
    private final IBooleanSubscriber brakeModeCheckbox;

    public enum StartPosition
    {
        Mid,
        Load,
        Guard
    }

    public enum AutoRoutine
    {
        None,
        Place,
        Taxi,
        Charge,
        OnePlusTaxi,
        OnePlusCharge,
        OnePlusPickup,
        OnePickupCharge,
        OnePlusOne,
        ThreePiece
    }

    public enum MotorMode
    {
        PercentOutput,
        PositionPID,
        VelocityPID,
        TrapezoidalMotionProfile,
    }

    /**
     * Initializes a new SmartDashboardSelectionManager
     */
    @Inject
    public SmartDashboardSelectionManager(
        IRobotProvider provider)
    {
        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser();
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("Taxi", AutoRoutine.Taxi);
        this.routineChooser.addObject("Charge", AutoRoutine.Charge);
        this.routineChooser.addObject("One Pickup", AutoRoutine.OnePlusPickup);
        this.routineChooser.addObject("One Plus Taxi", AutoRoutine.OnePlusTaxi);
        this.routineChooser.addObject("One Plus Charge", AutoRoutine.OnePlusCharge);
        this.routineChooser.addObject("One Pickup Charge", AutoRoutine.OnePickupCharge);
        this.routineChooser.addObject("One Plus One", AutoRoutine.OnePlusOne);
        this.routineChooser.addObject("Three Piece", AutoRoutine.ThreePiece);
        this.routineChooser.addObject("Place", AutoRoutine.Place);
        networkTableProvider.addChooser("Auto Routine", this.routineChooser);

        this.positionChooser = networkTableProvider.getSendableChooser();
        this.positionChooser.addDefault("middle", StartPosition.Mid);
        this.positionChooser.addObject("load", StartPosition.Load);
        this.positionChooser.addObject("guard", StartPosition.Guard);
        networkTableProvider.addChooser("Start Position", this.positionChooser);

        this.motorModeChooser = networkTableProvider.getSendableChooser();
        this.motorModeChooser.addDefault("percentOutput", MotorMode.PercentOutput);
        this.motorModeChooser.addObject("positionPID", MotorMode.PositionPID);
        this.motorModeChooser.addObject("velocityPID", MotorMode.VelocityPID);
        this.motorModeChooser.addObject("trapezoidalMotionProfile", MotorMode.TrapezoidalMotionProfile);
        networkTableProvider.addChooser("Motor Mode", this.motorModeChooser);

        this.kpSlider = networkTableProvider.getNumberSlider("kP", 0.0);
        this.kiSlider = networkTableProvider.getNumberSlider("kI", 0.0);
        this.kdSlider = networkTableProvider.getNumberSlider("kD", 0.0);
        this.kfSlider = networkTableProvider.getNumberSlider("kF", 0.0);
        this.kAccelSlider = networkTableProvider.getNumberSlider("kAccel", 0.0);
        this.kCruiseVelSlider = networkTableProvider.getNumberSlider("kCruiseVel", 0.0);

        this.invertOutputCheckbox = networkTableProvider.getCheckbox("invertOutput", false);
        this.brakeModeCheckbox = networkTableProvider.getCheckbox("useBrakeMode", false);
    }

    public StartPosition getSelectedStartPosition()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.positionChooser, StartPosition.Mid);
    }

    public AutoRoutine getSelectedAutoRoutine()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.routineChooser, AutoRoutine.None);
    }

    public MotorMode getSelectedMotorMode()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.motorModeChooser, MotorMode.PercentOutput);
    }

    public double getSelectedKP()
    {
        return this.kpSlider.get();
    }

    public double getSelectedKI()
    {
        return this.kiSlider.get();
    }

    public double getSelectedKD()
    {
        return this.kdSlider.get();
    }

    public double getSelectedKF()
    {
        return this.kfSlider.get();
    }

    public double getSelectedAcceleration()
    {
        return this.kAccelSlider.get();
    }

    public double getSelectedCruiseVelocity()
    {
        return this.kCruiseVelSlider.get();
    }

    public boolean getSelectedInvertOutput()
    {
        return this.invertOutputCheckbox.get();
    }

    public boolean getSelectedBrakeMode()
    {
        return this.brakeModeCheckbox.get();
    }

    private static <T> T GetSelectedOrDefault(ISendableChooser<T> chooser, T defaultValue)
    {
        T selected = chooser.getSelected();
        if (selected == null)
        {
            selected = defaultValue;
        }

        return selected;
    }
}
