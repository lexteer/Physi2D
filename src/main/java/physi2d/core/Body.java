package physi2d.core;

import physi2d.math.Vec2;
import physi2d.shapes.Shape2d;

public class Body {
    private Vec2 position;
    private double angle; // in radians
    private double mass;
    private double invMass;
    private double invInertia;
    private double gravityScale = 1.0;
    private double restitution = 0.2; // 0.0 - 1.0
    private double staticFriction = 0.5;
    private double dynamicFriction = 0.3;
    private double linearDamping = 0.01;
    private double angularDamping = 0.01;
    private double dragCoefficient = 0.1;
    private Vec2 velocity = Vec2.ZERO;
    private double angularVelocity; // rad/s
    private Vec2 force = Vec2.ZERO;
    private Shape2d shape;
    private boolean autoInertia;
    private double torque;

    // 0 mass - wont move
    // 0 inertia - wont rotate
    public Body(Vec2 position, double mass, double inertia, Shape2d shape) {
        this.position = position;
        this.shape = shape;
        this.mass = mass;
        this.invMass = (mass == 0) ? 0 : 1/mass;
        this.invInertia = (inertia == 0) ? 0 : 1/inertia;
        autoInertia = false;
    }

    public Body(Vec2 position, double mass, Shape2d shape) {
        this.position = position;
        this.shape = shape;
        this.mass = mass;
        this.invMass = (mass == 0) ? 0 : 1/mass;
        recomputeInertia();
        autoInertia = true;
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

    public void setMass(double mass) {
        this.mass = mass;
        this.invMass = (mass == 0) ? 0 : 1/mass;
        if (autoInertia) recomputeInertia();
    }

    public double getInvInertia() {
        return invInertia;
    }

    public void setInertia(double inertia) {
        this.invInertia = (inertia == 0) ? 0 : 1/inertia;
        autoInertia = false;
    }

    private void recomputeInertia() {
        this.invInertia = (mass == 0) ? 0 : 1 / shape.computeInertia(mass);
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

    public double getLinearDamping() {
        return linearDamping;
    }

    public void setLinearDamping(double linearDamping) {
        this.linearDamping = Math.clamp(linearDamping, 0.0, 0.9999);
    }

    public double getAngularDamping() {
        return angularDamping;
    }

    public void setAngularDamping(double angularDamping) {
        this.angularDamping = Math.clamp(angularDamping, 0.0, 0.9999);
    }

    public double getDragCoefficient() {
        return dragCoefficient;
    }

    public void setDragCoefficient(double dragCoefficient) {
        this.dragCoefficient = dragCoefficient;
    }

    public Shape2d getShape() {
        return shape;
    }

    public void setShape(Shape2d shape) {
        this.shape = shape;
        if (autoInertia) recomputeInertia();
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }



    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    public Vec2 getForce() {
        return force;
    }

    public void applyForce(Vec2 force) {
        this.force = this.force.add(force);
    }

    public void applyForceAtPoint(Vec2 force, Vec2 worldPoint) {
        applyForceAtOffset(force, worldPoint.sub(position));
    }

    public void applyForceAtOffset(Vec2 force, Vec2 offset) {
        applyForce(force);
        torque += offset.cross(force);
    }

    public double getTorque() {
        return torque;
    }

    public void applyTorque(double torque) {
        this.torque += torque;
    }

    public void clearForces() {
        this.force = Vec2.ZERO;
        this.torque = 0;
    }

    public Vec2 velocityAtPoint(Vec2 offset) {
        return velocity.add(new Vec2(-angularVelocity * offset.y(), angularVelocity * offset.x()));
    }
}
