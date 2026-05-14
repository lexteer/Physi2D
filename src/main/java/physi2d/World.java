package physi2d;

import physi2d.math.Vector2D;
import physi2d.physics.*;

import java.util.ArrayList;
import java.util.List;

public class World {
    private List<PhysicsObject> physicsObjectsList;
    private Vector2D gravity;

    public World() {
        gravity = new Vector2D(0, -980); // default
        physicsObjectsList = new ArrayList<>();
    }

    public void addObject(PhysicsObject object) {
        physicsObjectsList.add(object);
    }

    public List<PhysicsObject> getObjects() {
        return physicsObjectsList;
    }

    public void setGravity(Vector2D value) {
        this.gravity = value.copy();
    }

    public void step(double dt) {
        // apply gravity and integrate
        for (PhysicsObject object : physicsObjectsList) {
            RigidBody body = object.getBody();

            if (body.hasGravityOverride()) {
                body.applyForce(body.getOverriddenGravity());
            } else {
                body.applyForce(gravity);
            }

            body.integrate(dt);
        }

        // bread phase
        List<PhysicsObject[]> broadCollisions = BroadPhase.getPotentialCollisions(physicsObjectsList);

        // narrow phase + collision resolution
        for (PhysicsObject[] pair : broadCollisions) {
            PhysicsObject objectA = pair[0];
            PhysicsObject objectB = pair[1];

            CollisionManifold manifold = NarrowPhase.checkCollision(objectA, objectB);

            // resolution
            CollisionResolution.resolvePosition(objectA, objectB, manifold);
            CollisionResolution.resolveVelocity(objectA, objectB, manifold);
        }
    }
}
