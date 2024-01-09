package frc.lib.robotprovider;

import java.util.OptionalDouble;

public class PathPlannerWaypoint
{
    public final double x;
    public final double y;
    public final double heading;
    public final double orientation;

    public final OptionalDouble maxVelocity;
    public final OptionalDouble maxAcceleration;
    public final OptionalDouble maxAngularVelocity;
    public final OptionalDouble maxAngularAcceleration;

    public static int setOrientation(boolean isRed, boolean forward)
    {
        if (forward)
        {
            return isRed ? 180 : 0;
        }
        else
        {
            return isRed ? 0 : 180;
        }
    }

    /**
     * Creates a waypoint at position (x, y), assuming that the robot should be heading forward at this point and oriented forward.
     * @param x position (in inches)
     * @param y position (in inches)
     */
    public PathPlannerWaypoint(double x, double y)
    {
        this(x, y, 0.0);
    }

    /**
     * Creates a waypoint at position (x, y), traveling in the direction of the heading, facing the orientation, with an overridden velocity
     * @param x position (in inches)
     * @param y position (in inches)
     * @param heading travel direction (tangent, in degrees)
     */
    public PathPlannerWaypoint(double x, double y, double heading)
    {
        this(x, y, heading, 0.0);
    }

    /**
     * Creates a waypoint at position (x, y), traveling in the direction of the heading, facing the orientation, with an overridden velocity
     * @param point position (in inches)
     * @param heading travel direction (tangent, in degrees)
     * @param orientation facing direction (in degrees)
     */
    public PathPlannerWaypoint(Point2d point, double heading, double orientation)
    {
        this(
            point.x,
            point.y,
            heading,
            orientation,
            OptionalDouble.empty(),
            OptionalDouble.empty(),
            OptionalDouble.empty(),
            OptionalDouble.empty());
    }

    /**
     * Creates a waypoint at position (x, y), traveling in the direction of the heading, facing the orientation, with an overridden velocity
     * @param point position (in inches)
     * @param heading travel direction (tangent, in degrees)
     * @param orientation facing direction (in degrees)
     * @param velocityOverride while approaching this waypoint (in inches per second)
     */
    public PathPlannerWaypoint(
        Point2d point,
        double heading,
        double orientation,
        double maxVelocity,
        double maxAcceleration,
        double maxAngularVelocity,
        double maxAngularAcceleration)
    {
        this(
            point.x,
            point.y,
            heading,
            orientation,
            OptionalDouble.of(maxVelocity),
            OptionalDouble.of(maxAcceleration),
            OptionalDouble.of(maxAngularVelocity),
            OptionalDouble.of(maxAngularAcceleration));
    }

    /**
     * Creates a waypoint at position (x, y), traveling in the direction of the heading, facing the orientation, with an overridden velocity
     * @param x position (in inches)
     * @param y position (in inches)
     * @param heading travel direction (tangent, in degrees)
     * @param orientation facing direction (in degrees)
     */
    public PathPlannerWaypoint(double x, double y, double heading, double orientation)
    {
        this(
            x,
            y,
            heading,
            orientation,
            OptionalDouble.empty(),
            OptionalDouble.empty(),
            OptionalDouble.empty(),
            OptionalDouble.empty());
    }

    /**
     * Creates a waypoint at position (x, y), traveling in the direction of the heading, facing the orientation, with an overridden velocity
     * @param x position (in inches)
     * @param y position (in inches)
     * @param heading travel direction (tangent, in degrees)
     * @param orientation facing direction (in degrees)
     * @param velocityOverride while approaching this waypoint (in inches per second)
     */
    public PathPlannerWaypoint(
        double x,
        double y,
        double heading,
        double orientation,
        double maxVelocity,
        double maxAcceleration,
        double maxAngularVelocity,
        double maxAngularAcceleration)
    {
        this(
            x,
            y,
            heading,
            orientation,
            OptionalDouble.of(maxVelocity),
            OptionalDouble.of(maxAcceleration),
            OptionalDouble.of(maxAngularVelocity),
            OptionalDouble.of(maxAngularAcceleration));
    }

    /**
     * Creates a waypoint at position (x, y), traveling in the direction of the heading, facing the orientation, with an overridden velocity
     * @param x position (in inches)
     * @param y position (in inches)
     * @param heading travel direction (tangent, in degrees)
     * @param orientation facing direction (in degrees)
     * @param velocityOverride while approaching this waypoint (in inches per second)
     */
    private PathPlannerWaypoint(
        double x,
        double y,
        double heading,
        double orientation,
        OptionalDouble maxVelocity,
        OptionalDouble maxAcceleration,
        OptionalDouble maxAngularVelocity,
        OptionalDouble maxAngularAcceleration)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.orientation = orientation;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
    }
}