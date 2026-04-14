package physi2d.shapes;

import physi2d.math.Vector2D;

/**
 * Circle shape used in the OBB narrow collision search.
 * Only holds its radius.
 */
public class Circle implements Shape{
    private final double radius;

    public Circle(double radius) {
        this.radius = radius;
    }

    @Override
    public Vector2D[] getAxes() {
        return new Vector2D[0];
    }

    /**
     * Calculates (projects) the min and max (left and right) point
     * on a 1D plane (axis) in LOCAL space.
     * To be used for OBB collision check.
     *
     * @param axis Axis on which the min and max points are projected
     * @return Record class holding min and max value of the projection
     */
    @Override
    public Projection project(Vector2D axis) {
        return new Projection(-radius, radius);
    }

    /**
     * Calculates (projects) the min and max (left and right) point
     * on a 1D plane (axis) in WORLD space.
     * To be used for OBB collision check.
     *
     * @param axis Axis on which the min and max points are projected.
     * @param worldPosition Position of the RigidBody in world space.
     * @param bodyAngle Unused, circles are rotationally symmetric, value doesn't matter
     * @return Record class holding min and max value of the projection.
     */
    @Override
    public Projection project(Vector2D axis, Vector2D worldPosition, double bodyAngle) {
        double center = worldPosition.dotProduct(axis);
        return new Projection(center - radius, center + radius);
    }

    @Override
    public ShapeType getType() {
        return ShapeType.CIRCLE;
    }

    public double getRadius() {
        return radius;
    }
}
