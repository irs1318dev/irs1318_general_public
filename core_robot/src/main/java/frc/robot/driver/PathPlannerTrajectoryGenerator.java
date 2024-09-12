package frc.robot.driver;

import frc.lib.driver.TrajectoryManager;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerRotationTarget;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.robot.AutonLocManager;
import frc.robot.TuningConstants;

public class PathPlannerTrajectoryGenerator
{
    public static void generateTrajectories(TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        PathPlannerTrajectoryGenerator.generateTrajectories(false, trajectoryManager, pathPlanner);
        PathPlannerTrajectoryGenerator.generateTrajectories(true, trajectoryManager, pathPlanner);

        // ------------------------------- Macro paths --------------------------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-30.0, 0.0, 180.0, 0.0)),
                "goBackwards30in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-6.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
            "goBackwards15in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-12.0, 0.0, 180.0, 0.0)),
                "goBackwards15in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(9.0, 16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, 32.0, 0.0, 0.0)),
            "goLeft32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(9.0, -16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, -32.0, 0.0, 0.0)),
            "goRight32inForward18in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 11.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 22.0, 90.0, 0.0)),
            "goLeft22in");
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -11.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -22.0, 270.0, 0.0)),
            "goRight22in");
    }

    public static void generateTrajectories(boolean isRed, TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        AutonLocManager locManager = new AutonLocManager(isRed);

        // ----------------------> EXAMPLE PATH <-----------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.TANK_DRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P1, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0))),
            isRed ? "ExamplePathRed" : "ExamplePathBlue");
    }

    private static void addTrajectory(TrajectoryManager trajectoryManager, ITrajectory trajectory, String name)
    {
        // ExceptionHelpers.Assert(trajectory != null, "Adding null trajectory '%s'!", name);
        try
        {
            trajectoryManager.addTrajectory(name, trajectory);
        }
        catch (Exception ex)
        {
            System.err.println("Encountered exception generating path " + name + ": " + ex.toString());
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw ex;
            }
        }
    }
}