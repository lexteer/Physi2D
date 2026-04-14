package physi2d.physics;

import physi2d.math.Vector2D;
import physi2d.shapes.Circle;
import physi2d.shapes.Polygon;
import physi2d.shapes.Projection;
import physi2d.shapes.ShapeType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class NarrowPhase {
    public static CollisionManifold checkCollision(PhysicsObject a, PhysicsObject b) {
        ShapeType shapeA = a.getShape().getType();
        ShapeType shapeB = b.getShape().getType();
        ShapeType circle = ShapeType.CIRCLE;
        ShapeType polygon = ShapeType.POLYGON;

        if (shapeA == circle && shapeB == circle) {
            return checkCircleCircle(a, b);
        }
        else if (shapeA == polygon && shapeB == polygon) {
            return checkPolygonPolygon(a, b);
        }
        else if (shapeA == circle && shapeB == polygon){
            return checkCirclePolygon(a, b);
        }
        else if (shapeA == polygon && shapeB == circle) {
            return checkCirclePolygon(b, a);
        }

        throw new IllegalStateException("[NarrowPhase] Unknown shape type combination");
    }

    /**
     * A narrow phase OBB check of a collision case circle vs circle.
     *
     * @param a First physics object with a circle shape.
     * @param b Second physics object with a circle shape.
     * @return Manifold of the collision containing status of the
     * collision, the normal of the collision which is to be used to
     * push the objects apart and the depth of the collision.
     */
    private static CollisionManifold checkCircleCircle(PhysicsObject a, PhysicsObject b) {
        Vector2D centerA = a.getBody().getWorldPosition();
        Vector2D centerB = b.getBody().getWorldPosition();

        Circle circleA = (Circle) a.getShape();
        Circle circleB = (Circle) b.getShape();

        double radiiSum = circleA.getRadius() + circleB.getRadius();
        double distanceBetweenCenters = centerA.distance(centerB);
        boolean isCollision = distanceBetweenCenters < radiiSum;

        if (!isCollision) return new CollisionManifold(false, null, 0);

        Vector2D normal = centerB.copy().subtract(centerA);
        if (distanceBetweenCenters != 0) {
            normal.normalize();
        }
        double collisionDepth = radiiSum - distanceBetweenCenters;

        return new CollisionManifold(true, normal, collisionDepth);
    }

    /**
     * A narrow phase OBB check of a collision case circle vs polygon.
     *
     * @param circle Circle shape object
     * @param polygon Polygon shape object
     * @return Manifold of the collision containing status of the
     * collision, the normal of the collision which is to be used to
     * push the objects apart and the depth of the collision.
     */
    private static CollisionManifold checkCirclePolygon(PhysicsObject circle, PhysicsObject polygon) {
        RigidBody polyBody = polygon.getBody();
        double polyAngle = polyBody.getAngle();

        RigidBody circBody = circle.getBody();
        double circAngle = circBody.getAngle();

        Polygon polygonShape = (Polygon) polygon.getShape();
        Circle circleShape = (Circle) circle.getShape();

        Vector2D polyPos = polyBody.getWorldPosition();
        Vector2D circPos = circBody.getWorldPosition();

        // get axis from the closest poly vertex to the circle center, normalized
        Vector2D closestVertex = getClosestVertex((Polygon) polygon.getShape(), polygon.getBody(), circPos);
        Vector2D polyToCircleAxis = circPos.copy().subtract(closestVertex).normalize();

        Vector2D[] polyAxes = getWorldAxes(polygon);
        List<Vector2D> axesToTest = new ArrayList<>(Arrays.asList(polyAxes));
        axesToTest.add(polyToCircleAxis);

        Vector2D smallestOverlapAxis = new Vector2D();
        double minOverlapDepth = Double.MAX_VALUE;

        for (Vector2D axis : axesToTest) {
            Projection projectionA = polygonShape.project(axis, polyPos, polyAngle);
            Projection projectionB = circleShape.project(axis, circPos, circAngle);

            boolean overlap = projectionA.overlaps(projectionB);

            if (!overlap) return new CollisionManifold(false, null, 0);

            double overlapDepth = projectionA.getOverlapDepth(projectionB);
            // track the axis with the smallest overlap
            if (overlapDepth < minOverlapDepth) {
                minOverlapDepth = overlapDepth;
                smallestOverlapAxis = axis;
            }
        }

        // check if the normal is facing the correct direction and flip it if not
        if (circPos.copy().subtract(closestVertex).dotProduct(smallestOverlapAxis) < 0) {
            smallestOverlapAxis = smallestOverlapAxis.multiply(-1);
        }

        return new CollisionManifold(true, smallestOverlapAxis.normalize(), minOverlapDepth);
    }

    /**
     * A narrow phase OBB check of a collision case polygon vs polygon.
     *
     * @param a First physics object with a polygon shape.
     * @param b Second physics object with a polygon shape.
     * @return Manifold of the collision containing status of the
     * collision, the normal of the collision which is to be used to
     * push the objects apart and the depth of the collision.
     */
    private static CollisionManifold checkPolygonPolygon(PhysicsObject a, PhysicsObject b) {
        RigidBody bodyA = a.getBody();
        RigidBody bodyB = b.getBody();

        double angleA = bodyA.getAngle();
        double angleB = bodyB.getAngle();

        Vector2D posA = bodyA.getWorldPosition();
        Vector2D posB = bodyB.getWorldPosition();

        Polygon polygonA = (Polygon) a.getShape();
        Polygon polygonB = (Polygon) b.getShape();

        Vector2D[] axesA = getWorldAxes(a);
        Vector2D[] axesB = getWorldAxes(b);

        // project both shapes onto all axes and check for overlap
        List<Vector2D> axes = new ArrayList<>(Arrays.asList(axesA));
        axes.addAll(Arrays.asList(axesB));

        Vector2D smallestOverlapAxis = new Vector2D();
        double minOverlapDepth = Double.MAX_VALUE;

        for (Vector2D axis : axes) {
            Projection projectionA = polygonA.project(axis, posA, angleA);
            Projection projectionB = polygonB.project(axis, posB, angleB);

            boolean overlap = projectionA.overlaps(projectionB);

            if (!overlap) return new CollisionManifold(false, null, 0);

            double overlapDepth = projectionA.getOverlapDepth(projectionB);
            // track the axis with the smallest overlap
            if (overlapDepth < minOverlapDepth) {
                minOverlapDepth = overlapDepth;
                smallestOverlapAxis = axis;
            }
        }

        // check if the normal is facing the correct direction and flip it if not
        if (posB.copy().subtract(posA).dotProduct(smallestOverlapAxis) < 0) {
            smallestOverlapAxis = smallestOverlapAxis.multiply(-1);
        }

        return new CollisionManifold(true, smallestOverlapAxis.normalize(), minOverlapDepth);
    }

    /**
     * Helper method for converting local axes to world axes by rotating them
     * by the objects angle.
     *
     * @param obj Object used to get the shapes axes.
     * @return Array of world axes converted from local.
     */
    private static Vector2D[] getWorldAxes(PhysicsObject obj) {
        Vector2D[] axes = obj.getShape().getAxes();

        double angle = obj.getBody().getAngle();

        for (Vector2D axis : axes) {
            axis.rotate(angle);
        }

        return axes;
    }

    /**
     * Helper method for getting the closest polygon vertex to the circle shapes center.
     * Used in the circle vs polygon OBB check.
     *
     * @param polygon Polygon shape used for its vertices.
     * @param polyBody Polygon body used to get its position and angle to transform vertices.
     * @param circleCenter Center point of the circle shape.
     * @return Closest polygon vertex to the center of the circle.
     */
    private static Vector2D getClosestVertex(Polygon polygon, RigidBody polyBody, Vector2D circleCenter) {
        Vector2D[] vertices = polygon.getVertices();
        Vector2D closest = vertices[0].copy().rotate(polyBody.getAngle()).add(polyBody.getWorldPosition());

        for (int i = 1; i < vertices.length; i++) {
            Vector2D transformed = vertices[i].copy().rotate(polyBody.getAngle()).add(polyBody.getWorldPosition());

            if (transformed.distance(circleCenter) < closest.distance(circleCenter)) {
                closest = transformed;
            }
        }

        return closest;
    }
}
