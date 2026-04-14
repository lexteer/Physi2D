package physi2d.physics;

import physi2d.math.Vector2D;
import physi2d.shapes.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Used for calculating the AABB collisions. Wraps shapes like circle
 * and polygons in a AABB shape and tests the collisions with them.
 * To be used before doing a narrow phase collision detection to rule
 * out a lot of collision checks.
 */
public class BroadPhase {
    /**
     * Calls either cumputeCircle() or computePolygon
     * based on the shape.
     *
     * @param body Rigid body on which the AABB shape is computed
     * @param shape Needed to know which method to call
     * @return The AABB wrapped shape
     */
    public static AABB computeAABB(RigidBody body, Shape shape) {
        if (shape.getType() == ShapeType.CIRCLE) {
            return computeCircle(body, (Circle) shape);
        } else {
            return computePolygon(body, (Polygon) shape);
        }
    }

    /**
     * Calculates the AABB shape needed to wrap the circle tightly.
     * Uses the world position of the body in the calculation.
     *
     * @param body Rigid body on which the AABB shape is computed
     * @param circle The shape used to get the radius
     * @return Tight AABB shape
     */
    private static AABB computeCircle(RigidBody body, Circle circle) {
        Vector2D position = body.getWorldPosition();

        double minX = position.x - circle.getRadius();
        double maxX = position.x + circle.getRadius();
        double minY = position.y - circle.getRadius();
        double maxY = position.y + circle.getRadius();

        return new AABB(minX, maxX, minY, maxY);
    }

    /**
     * Calculates the AABB shape needed to wrap the polygon tightly.
     * Uses the world position of the body in the calculation.
     *
     * @param body Rigid body on which the AABB shape is computed
     * @param polygon Used to get all vertices
     * @return Tight AABB shape
     */
    private static AABB computePolygon(RigidBody body, Polygon polygon) {
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;

        Vector2D[] vertices = polygon.getVertices();

        for (Vector2D vertex : vertices) {
            // transform to word space
            Vector2D v = vertex.copy();
            v.rotate(body.getAngle());
            v.add(body.getWorldPosition());

            double x = v.x;
            double y = v.y;

            if (x < minX) {
                minX = x;
            } else if (x > maxX) {
                maxX = x;
            }

            if (y < minY) {
                minY = y;
            } else if (y > maxY) {
                maxY = y;
            }
        }

        return new AABB(minX, maxX, minY, maxY);
    }

    /**
     * Computes a broad collision search using the AABB shapes computed from
     * the previous computeAABB() method.
     * Do a narrow collision search on the output of this method
     * to catch the actual collisions.
     * This method may contain false positives, but never
     * false negatives.
     *
     * @param objects All physics objects, each one having a rigid body and a shape
     * @return List of pairs(2) of rigid bodies that MAY be colliding
     */
    public static List<PhysicsObject[]> getPotentialCollisions(List<PhysicsObject> objects) {
        List<PhysicsObject[]> collidedPairs = new ArrayList<>();
        List<AABB> aabbShapes = new ArrayList<>();

        // compute all aabb-s
        for (PhysicsObject object : objects) {
            aabbShapes.add(computeAABB(object.getBody(), object.getShape()));
        }

        // check each pair ob aabb shapes for collision
        for (int i = 0; i < objects.size(); i++) {
            AABB aabbShape = aabbShapes.get(i);

            for (int j = i + 1; j < objects.size(); j++) {
                AABB aabbShape2 = aabbShapes.get(j);

                if (aabbShape.overlaps(aabbShape2)) {
                    PhysicsObject[] pair = {objects.get(i), objects.get(j)};
                    collidedPairs.add(pair);
                }
            }
        }

        return collidedPairs;
    }
}
