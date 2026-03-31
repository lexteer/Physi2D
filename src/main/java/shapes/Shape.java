package shapes;

import math.Vector2D;

/**
 * To be used on all shapes for the OBB collision checks.
 */
public interface Shape {
    /**
     * @return - All edge normals of the shape
     */
    Vector2D[] getAxes();

    /**
     * Project the shape onto a 1D plane and extract
     * the min and max points.
     *
     * @param axis - axis onto which the shape is projected
     * @return - min and max value of the projection
     */
    Projection project(Vector2D axis);

    /**
     * Used in the collision checks
     *
     * @return - simple enum with CIRCLE or POLYGON
     */
    ShapeType getType();
}
