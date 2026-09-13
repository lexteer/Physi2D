package physi2d.core;

import physi2d.math.Vec2;
import physi2d.shapes.Shape2d;

public class Body {
    private Vec2 position;
    private double invMass;
    private double gravityScale = 1.0;
    private double restitution = 0.2; // 0.0 - 1.0
    private double staticFriction = 0.5;
    private double dynamicFriction = 0.3;
    private Vec2 velocity = Vec2.ZERO;
    private Vec2 force = Vec2.ZERO;
    private Shape2d shape;

    public Body(Vec2 position, double mass, Shape2d shape) {
        this.position = position;
        this.invMass = (mass == 0) ? 0 : 1/mass;
        this.shape = shape;
    }

    public Vec2 getPosition() {
        return position;
    }

    public void setPosition(Vec2 position) {
        this.position = position;
    }

    public Vec2 getVelocity() {
        return velocity;
    }

    public void setVelocity(Vec2 velocity) {
        this.velocity = velocity;
    }

    public double getInvMass() {
        return invMass;
    }

    public double getGravityScale() {
        return gravityScale;
    }

    public void setGravityScale(double gravityScale) {
        this.gravityScale = gravityScale;
    }

    public double getRestitution() {
        return restitution;
    }

    public void setRestitution(double restitution) {
        this.restitution = Math.clamp(restitution, 0.0, 1.0);
    }

    public double getStaticFriction() {
        return staticFriction;
    }

    public void setStaticFriction(double staticFriction) {
        this.staticFriction = staticFriction;
    }

    public double getDynamicFriction() {
        return dynamicFriction;
    }

    public void setDynamicFriction(double dynamicFriction) {
        this.dynamicFriction = dynamicFriction;
    }

    public Shape2d getShape() {
        return shape;
    }

    public Vec2 getForce() {
        return force;
    }

    public void applyForce(Vec2 force) {
        this.force = this.force.add(force);
    }

    public void clearForce() {
        this.force = Vec2.ZERO;
    }
}
