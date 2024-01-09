package frc.lib.robotprovider;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.helpers.Helpers;

public class PathPlannerWrapper implements IPathPlanner
{
    public PathPlannerWrapper()
    {
    }

    @Override
    public ITrajectory loadTrajectory(String name, double maxVelocity, double maxAcceleration)
    {
        return this.loadTrajectory(name, maxVelocity, maxAcceleration, false);
    }

    @Override
    public ITrajectory loadTrajectory(String name, double maxVelocity, double maxAcceleration, boolean reversed)
    {
        PathPlannerPath path = PathPlannerPath.fromPathFile(name);
        return new PathPlannerTrajectoryWrapper(path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(0.0)));
    }

    @Override    
    public ITrajectory buildTrajectory(
        double maxVelocity,
        double maxAcceleration,
        double maxAngularVelocity,
        double maxAngularAcceleration,
        double endRotation,
        PathPlannerWaypoint... waypoints)
    {
        PathConstraints constraints = new PathConstraints(
            maxVelocity * Helpers.METERS_PER_INCH,
            maxAcceleration * Helpers.METERS_PER_INCH,
            maxAngularVelocity * Helpers.DEGREES_TO_RADIANS,
            maxAngularAcceleration * Helpers.DEGREES_TO_RADIANS);

        PathPlannerPath path = PathPlannerPath.fromPathPoints(this.convertWaypoints(waypoints), constraints, new GoalEndState(0.0, Rotation2d.fromDegrees(endRotation)));

        return new PathPlannerTrajectoryWrapper(
            new PathPlannerTrajectory(path, new ChassisSpeeds(), new Rotation2d()));
    }

    private PathPoint convertWaypoint(PathPlannerWaypoint waypoint)
    {
        PathConstraints constraints = null;
        if (waypoint.maxVelocity.isPresent() &&
            waypoint.maxAcceleration.isPresent() &&
            waypoint.maxAngularVelocity.isPresent() &&
            waypoint.maxAngularAcceleration.isPresent())
        {
            constraints = new PathConstraints(
                waypoint.maxVelocity.getAsDouble() * Helpers.METERS_PER_INCH,
                waypoint.maxAcceleration.getAsDouble() * Helpers.METERS_PER_INCH,
                waypoint.maxAngularVelocity.getAsDouble() * Helpers.DEGREES_TO_RADIANS,
                waypoint.maxAngularAcceleration.getAsDouble() * Helpers.DEGREES_TO_RADIANS);
        }

        return new PathPoint(
            new Translation2d(waypoint.x * Helpers.METERS_PER_INCH, waypoint.y * Helpers.METERS_PER_INCH),
            new RotationTarget(waypoint.orientation, Rotation2d.fromDegrees(waypoint.orientation)),
            constraints);
    }

    private List<PathPoint> convertWaypoints(PathPlannerWaypoint[] waypoints)
    {
        if (waypoints == null)
        {
            return new ArrayList<PathPoint>(0);
        }

        ArrayList<PathPoint> points = new ArrayList<PathPoint>(waypoints.length);
        for (int i = 0; i < waypoints.length; i++)
        {
            points.add(this.convertWaypoint(waypoints[i]));
        }

        return points;
    }

    private class PathPlannerTrajectoryWrapper implements ITrajectory
    {
        private final PathPlannerTrajectory wrappedObject;

        PathPlannerTrajectoryWrapper(PathPlannerTrajectory wrappedObject)
        {
            this.wrappedObject = wrappedObject;
        }

        @Override
        public double getDuration()
        {
            return this.wrappedObject.getTotalTimeSeconds();
        }

        @Override
        public TrajectoryState get(double time)
        {
            State state = (State)this.wrappedObject.sample(time);
            return new TrajectoryState(
                state.positionMeters.getX() * Helpers.INCHES_PER_METER,
                state.positionMeters.getY() * Helpers.INCHES_PER_METER,
                state.targetHolonomicRotation.getDegrees(),
                state.heading.getCos() * state.velocityMps * Helpers.INCHES_PER_METER,
                state.heading.getSin() * state.velocityMps * Helpers.INCHES_PER_METER,
                state.holonomicAngularVelocityRps.isPresent() ? state.holonomicAngularVelocityRps.get() * Helpers.RADIANS_TO_DEGREES : 0.0);
        }
    }
}
