package physics;

import com.github.lexteer.math.Vector2D;
import com.github.lexteer.physics.BroadPhase;
import com.github.lexteer.physics.RigidBody;
import org.junit.jupiter.api.Test;
import com.github.lexteer.shapes.AABB;
import com.github.lexteer.shapes.Circle;
import com.github.lexteer.shapes.Polygon;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class BroadPhaseTest {

    @Test
    void circleAABBComputation() {
        RigidBody rigidBody = new RigidBody(3, 5, 10, 10);

        Circle circle = new Circle(2);

        AABB aabb = BroadPhase.computeAABB(rigidBody, circle);

        assertEquals(1, aabb.getMinX(), 1e-9);
        assertEquals(5, aabb.getMaxX(), 1e-9);
        assertEquals(3, aabb.getMinY(), 1e-9);
        assertEquals(7, aabb.getMaxY(), 1e-9);
    }

    @Test
    void polygonNoRotationComputation() {
        RigidBody rigidBody = new RigidBody(2, 2, 10, 10);

        Vector2D[] vertices = {
                new Vector2D(3, 3),
                new Vector2D(3, 7),
                new Vector2D(8, 7),
                new Vector2D(8, 3)
        };

        Polygon polygon = new Polygon(vertices);

        AABB aabb = BroadPhase.computeAABB(rigidBody, polygon);

        assertEquals(5, aabb.getMinX(), 1e-9);
        assertEquals(10, aabb.getMaxX(), 1e-9);
        assertEquals(5, aabb.getMinY(), 1e-9);
        assertEquals(9, aabb.getMaxY(), 1e-9);
    }

    @Test
    void polygonRotatedComputation() {
        RigidBody rigidBody = new RigidBody(0, 0, 10, 10);

        rigidBody.rotate(Math.PI / 4);

        Vector2D[] vertices = {
                new Vector2D(-1, -1),
                new Vector2D(1, -1),
                new Vector2D(1, 1),
                new Vector2D(-1, 1)
        };

        Polygon polygon = new Polygon(vertices);

        AABB aabb = BroadPhase.computeAABB(rigidBody, polygon);

        assertEquals(-Math.sqrt(2), aabb.getMinX(), 1e-3);
        assertEquals(Math.sqrt(2), aabb.getMaxX(), 1e-3);
        assertEquals(-Math.sqrt(2), aabb.getMinY(), 1e-3);
        assertEquals(Math.sqrt(2), aabb.getMaxY(), 1e-3);
    }

    @Test
    void aabbOverlappingPolygonTest() {
        // object 1
        RigidBody body1 = new RigidBody(3, 3, 10, 10);
        Vector2D[] vertices1 = {
                new Vector2D(-1, -1),
                new Vector2D(1, -1),
                new Vector2D(1, 1),
                new Vector2D(-1, 1)
        };
        Polygon shape1 = new Polygon(vertices1);
        PhysicsObject object1 = new PhysicsObject(body1, shape1);

        // object 2 overlaps object 1
        RigidBody body2 = new RigidBody(4, 4, 10, 10);
        Vector2D[] vertices2 = {
                new Vector2D(-1, -1),
                new Vector2D(1, -1),
                new Vector2D(1, 1),
                new Vector2D(-1, 1)
        };
        Polygon shape2 = new Polygon(vertices2);
        PhysicsObject object2 = new PhysicsObject(body2, shape2);

        List<PhysicsObject> allObjects = new ArrayList<>();
        allObjects.add(object1);
        allObjects.add(object2);

        // check collisions
        List<PhysicsObject[]> listOfAabbCollisions = BroadPhase.getPotentialCollisions(allObjects);

        assertEquals(1, listOfAabbCollisions.size(), 0);
    }

    @Test
    void aabbNotOverlappingPolygonTest() {
        // object 1
        RigidBody body1 = new RigidBody(3, 3, 10, 10);
        Vector2D[] vertices1 = {
                new Vector2D(-1, -1),
                new Vector2D(1, -1),
                new Vector2D(1, 1),
                new Vector2D(-1, 1)
        };
        Polygon shape1 = new Polygon(vertices1);
        PhysicsObject object1 = new PhysicsObject(body1, shape1);

        RigidBody body2 = new RigidBody(8, 8, 10, 10);
        Vector2D[] vertices2 = {
                new Vector2D(-1, -1),
                new Vector2D(1, -1),
                new Vector2D(1, 1),
                new Vector2D(-1, 1)
        };
        Polygon shape2 = new Polygon(vertices2);
        PhysicsObject object2 = new PhysicsObject(body2, shape2);

        List<PhysicsObject> allObjects = new ArrayList<>();
        allObjects.add(object1);
        allObjects.add(object2);

        // check collisions
        List<PhysicsObject[]> listOfAabbCollisions = BroadPhase.getPotentialCollisions(allObjects);

        assertEquals(0, listOfAabbCollisions.size(), 0);
    }

    @Test
    void aabbOverlappingTest() {
        // object 1
        RigidBody body1 = new RigidBody(3, 3, 10, 10);
        Circle shape1 = new Circle(2);
        PhysicsObject object1 = new PhysicsObject(body1, shape1);

        // object 2 overlaps object 1
        RigidBody body2 = new RigidBody(5, 5, 10, 10);
        Vector2D[] vertices2 = {
                new Vector2D(-1, -1),
                new Vector2D(1, -1),
                new Vector2D(1, 1),
                new Vector2D(-1, 1)
        };
        Polygon shape2 = new Polygon(vertices2);
        PhysicsObject object2 = new PhysicsObject(body2, shape2);

        List<PhysicsObject> allObjects = new ArrayList<>();
        allObjects.add(object1);
        allObjects.add(object2);

        // check collisions
        List<PhysicsObject[]> listOfAabbCollisions = BroadPhase.getPotentialCollisions(allObjects);

        assertEquals(1, listOfAabbCollisions.size(), 0);
    }
}
