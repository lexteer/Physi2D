package physi2d.shapes;

import physi2d.math.Vector2D;

/**
 * To be used on all shapes for the OBB collision checks.
 */
public interface Shape {
    /**
     * @return All edge normals of the shape
     */
    Vector2D[] getAxes();

    /**
     * Project the shape onto a 1D plane and extract
     * the min and max points.
     *
     * @param axis Axis onto which the shape is projected
     * @return Min and max value of the projection
     */
    Projection project(Vector2D axis);

    Projection project(Vector2D axis, Vector2D worldPosition, double bodyAngle);

    /**
     * Used in the collision checks
     *
     * @return Simple enum with CIRCLE or POLYGON
     */
    ShapeType getType();
}
