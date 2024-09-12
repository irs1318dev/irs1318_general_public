package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.robotprovider.*;

@Singleton
public class SmartDashboardSelectionManager
{
    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;
    private final ISendableChooser<PriorityPickupSide> pickupChooser;

    public enum StartPosition
    {
        None,
        Something,
    }

    public enum AutoRoutine
    {
        None,
        Something,
    }

    public enum PriorityPickupSide
    {
        None,
        Something,
    }

    /**
     * Initializes a new SmartDashboardSelectionManager
     */
    @Inject
    public SmartDashboardSelectionManager(
        IRobotProvider provider)
    {
        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser("Auto Routine");
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("Something", AutoRoutine.Something);

        this.positionChooser = networkTableProvider.getSendableChooser("Start Position");
        this.positionChooser.addDefault("None", StartPosition.None);
        this.positionChooser.addObject("Something", StartPosition.Something);

        this.pickupChooser = networkTableProvider.getSendableChooser("Pickup Chooser");
        this.pickupChooser.addDefault("None", PriorityPickupSide.None);
        this.pickupChooser.addObject("Something", PriorityPickupSide.Something);
    }

    public StartPosition getSelectedStartPosition()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.positionChooser, StartPosition.None);
    }

    public AutoRoutine getSelectedAutoRoutine()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.routineChooser, AutoRoutine.None);
    }

    public PriorityPickupSide getPickupSide()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.pickupChooser, PriorityPickupSide.None);
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
