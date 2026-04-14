package com.github.lexteer.shapes;

import com.github.lexteer.math.Vector2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Polygon shape that can be used to create many different shapes
 * using vertices.
 * Has origin at 0,0 so when creating an instance of this class
 * make sure you are creating is centered at 0,0.
 * To be used for narrow phase collision search.
 */
public class Polygon implements Shape{
    private final Vector2D[] vertices;

    public Polygon(Vector2D[] vertices) {
        this.vertices = vertices;
    }

    // returns edge normals

    /**
     * To be used for OBB collision checks.
     * Rotates all the edges by 90 degrees to get their Normals.
     *
     * @return List of all edge normals
     */
    @Override
    public Vector2D[] getAxes() {
        List<Vector2D> edges = getEdges();
        Vector2D[] normals = new Vector2D[edges.size()];

        for (int i = 0; i < edges.size(); i++) {
            Vector2D edge = edges.get(i);
            normals[i] = new Vector2D(-edge.y, edge.x); // rotate 90 degrees
        }

        return normals;
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
        double min = vertices[0].dotProduct(axis);
        double max = min;

        for (int i = 1; i < vertices.length; i++) {
            double projectionPoint = vertices[i].dotProduct(axis);

            if (projectionPoint < min) {
                min = projectionPoint;
            } else if (projectionPoint > max) {
                max = projectionPoint;
            }
        }

        return new Projection(min, max);
    }

    /**
     * Calculates (projects) the min and max (left and right) point
     * on a 1D plane (axis) in WORLD space.
     * To be used for OBB collision check.
     *
     * @param axis Axis on which the min and max points are projected.
     * @param worldPosition Used to translate all vertices to world space before projecting.
     * @param bodyAngle Used to rotate all vertices before projecting.
     * @return Record class holding min and max value of the projection.
     */
    @Override
    public Projection project(Vector2D axis, Vector2D worldPosition, double bodyAngle) {
        double min = vertices[0].copy().rotate(bodyAngle).add(worldPosition).dotProduct(axis);
        double max = min;

        for (int i = 1; i < vertices.length; i++) {
            Vector2D vertex = vertices[i].copy().rotate(bodyAngle).add(worldPosition);

            double projectionPoint = vertex.dotProduct(axis);

            if (projectionPoint < min) {
                min = projectionPoint;
            } else if (projectionPoint > max) {
                max = projectionPoint;
            }
        }

        return new Projection(min, max);
    }

    @Override
    public ShapeType getType() {
        return ShapeType.POLYGON;
    }

    /**
     * From a list of vertices calculates all edges
     * and displays them as vectors2D.
     *
     * @return List of all edges
     */
    private List<Vector2D> getEdges() {
        List<Vector2D> edges = new ArrayList<>();

        for (int i = 0; i < vertices.length; i++) {
            Vector2D firstVector = vertices[i].copy();
            Vector2D secondVector;

            if (i == vertices.length - 1) {
                secondVector = vertices[0].copy();
            } else {
                secondVector = vertices[i + 1].copy();
            }

            edges.add(secondVector.subtract(firstVector));
        }

        return edges;
    }

    public Vector2D[] getVertices() {
        return Arrays.copyOf(vertices, vertices.length);
    }
}
