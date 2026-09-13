package physi2d.core;

import physi2d.collision.CollisionDetection;
import physi2d.collision.CollisionManifold;
import physi2d.collision.CollisionResolution;
import physi2d.math.Vec2;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class World {
    private Vec2 gravity = new Vec2(0, -9.81);
    private List<Body> bodies;

    private double fluidDensity = 0.0;

    public World() {
        bodies = new ArrayList<>();
    }

    public void step(double dt) {
        for (Body body : bodies) {
            double iMass = body.getInvMass();

            // apply gravity
            double gravityScale = body.getGravityScale();
            if (iMass != 0 && !gravity.equals(Vec2.ZERO) && gravityScale != 0.0) {
                double mass = 1 / iMass;
                Vec2 gravityForce = gravity.mult(mass);
                body.applyForce(gravityForce.mult(gravityScale));
            }

            // fluid drag
            if (iMass != 0 && fluidDensity != 0) {
                Vec2 velocity = body.getVelocity();
                double dragCoefficient = body.getDragCoefficient();
                body.applyForce(velocity.mult(-fluidDensity * dragCoefficient));
            }

            integrate(body, dt);
            body.clearForce();
        }
        checkCollisionsAndResolve();
    }

    private void integrate(Body body, double dt) {
        Vec2 force = body.getForce();
        Vec2 acceleration = force.mult(body.getInvMass());

        Vec2 velocityChange = acceleration.mult(dt);
        body.setVelocity(body.getVelocity().add(velocityChange));
        body.setVelocity(body.getVelocity().mult(Math.pow(1.0 - body.getLinearDamping(), dt)));

        Vec2 posChange = body.getVelocity().mult(dt);
        body.setPosition(body.getPosition().add(posChange));
    }

    private void checkCollisionsAndResolve() {
        for (int i = 0; i < bodies.size(); i++) {
            Body a = bodies.get(i);
            for (int j = i + 1; j < bodies.size(); j++) {
                Body b = bodies.get(j);

                Optional<CollisionManifold> manifold = CollisionDetection.circleCircle(a, b);
                manifold.ifPresent(CollisionResolution::resolve);
            }
        }
    }

    public Vec2 getGravity() {
        return gravity;
    }

    public void setGravity(Vec2 gravity) {
        this.gravity = gravity;
    }

    public void addBody(Body body) {
        bodies.add(body);
    }

    public void removeBody(Body body) {
        bodies.remove(body);
    }

    public List<Body> getBodies() {
        return bodies;
    }

    public double getFluidDensity() {
        return fluidDensity;
    }

    public void setFluidDensity(double fluidDensity) {
        this.fluidDensity = fluidDensity;
    }
}
