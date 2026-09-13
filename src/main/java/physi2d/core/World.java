package physi2d.core;

import physi2d.math.Vec2;

import java.util.ArrayList;
import java.util.List;

public class World {
    private Vec2 gravity = new Vec2(0, -9.81);
    private List<Body> bodies;

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

            integrate(body, dt);
            body.clearForce();
        }
    }

    private void integrate(Body body, double dt) {
        Vec2 force = body.getForce();
        Vec2 acceleration = force.mult(body.getInvMass());

        Vec2 velocityChange = acceleration.mult(dt);
        body.setVelocity(body.getVelocity().add(velocityChange));

        Vec2 posChange = body.getVelocity().mult(dt);
        body.setPosition(body.getPosition().add(posChange));
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
}
