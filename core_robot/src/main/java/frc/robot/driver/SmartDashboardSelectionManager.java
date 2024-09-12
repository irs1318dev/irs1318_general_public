package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.robotprovider.*;

@Singleton
public class SmartDashboardSelectionManager
{
    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;

    public enum StartPosition
    {
        Center,
        Left,
        Right
    }

    public enum AutoRoutine
    {
        None,
        PathA,
        PathB,
        Slalom,
        Barrel,
        Bounce,
        ShootAndTrenchShoot,
        ShootAndShieldShoot,
        ShootAndMove,
        Shoot,
        Move,
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
        this.routineChooser.addObject("PathA", AutoRoutine.PathA);
        this.routineChooser.addObject("PathB", AutoRoutine.PathB);
        this.routineChooser.addObject("Slalom", AutoRoutine.Slalom);
        this.routineChooser.addObject("Barrel", AutoRoutine.Barrel);
        this.routineChooser.addObject("Bounce", AutoRoutine.Bounce);
        this.routineChooser.addObject("ShootAndTrenchShoot", AutoRoutine.ShootAndTrenchShoot);
        this.routineChooser.addObject("ShootAndShieldShoot", AutoRoutine.ShootAndShieldShoot);
        this.routineChooser.addObject("ShootAndMove", AutoRoutine.ShootAndMove);
        this.routineChooser.addObject("Shoot", AutoRoutine.Shoot);
        this.routineChooser.addObject("Move", AutoRoutine.Move);

        this.positionChooser = networkTableProvider.getSendableChooser("Start Position");
        this.positionChooser.addDefault("Center", StartPosition.Center);
        this.positionChooser.addObject("Left", StartPosition.Left);
        this.positionChooser.addObject("Right", StartPosition.Right);
    }

    public StartPosition getSelectedStartPosition()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.positionChooser, StartPosition.Center);
    }

    public AutoRoutine getSelectedAutoRoutine()
    {
        return SmartDashboardSelectionManager.GetSelectedOrDefault(this.routineChooser, AutoRoutine.None);
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
