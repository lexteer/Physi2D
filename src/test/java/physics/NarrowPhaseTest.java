package physics;

import com.github.physi2d.math.Vector2D;
import com.github.physi2d.physics.CollisionManifold;
import com.github.physi2d.physics.NarrowPhase;
import com.github.physi2d.physics.PhysicsObject;
import com.github.physi2d.physics.RigidBody;
import org.junit.jupiter.api.Test;
import com.github.physi2d.shapes.Circle;
import com.github.physi2d.shapes.Polygon;

import static org.junit.jupiter.api.Assertions.*;

public class NarrowPhaseTest {

    @Test
    void circleVsCircleCollision() {
        RigidBody bodyA = new RigidBody(2, 2, 10, 10);
        Circle circleA = new Circle(1);
        PhysicsObject a = new PhysicsObject(bodyA, circleA);

        RigidBody bodyB = new RigidBody(3.5, 2, 10, 10);
        Circle circleB = new Circle(1);
        PhysicsObject b = new PhysicsObject(bodyB, circleB);

        CollisionManifold test = NarrowPhase.checkCollision(a, b);

        assertTrue(test.isColliding());
        assertEquals(0.5, test.getDepth(), 1e-9);
        assertEquals(1, test.getNormal().x, 1e-9);
        assertEquals(0, test.getNormal().y, 1e-9);
    }

    @Test
    void polyVsPolyCollision() {
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

        CollisionManifold test = NarrowPhase.checkCollision(object1, object2);

        assertTrue(test.isColliding());
    }
}
