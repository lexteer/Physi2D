package physics;

import math.Vector2D;

/**
 * Rigid body represents a single simulated 2D body.
 * Stores position, velocity and rotation state, and advances
 * them each frame via integrate().
 */
public class RigidBody {
    // translation
    private Vector2D worldPosition;
    private Vector2D velocity;
    private Vector2D force;

    // rotation
    private double angle;
    private double angularVelocity;
    private double torque;

    // physical properties
    private double inverseMass;
    private double inverseInertia;

    private double oldInverseMass;
    private double oldInverseInertia;

    private static boolean rotationLock = false;

    /**
     * Transforms mass and inertia to their inverted counterparts
     * for optimization.
     * Default values are set.
     * Throws an exception if any of the arguments are below 1.
     *
     * @param mass Stored as 1/mass to avoid repeated division during
     * integration. A value of 0 represents infinite mass (immovable body).
     * @param inertia The rotational equivalent of inverse mass. Resistance to rotational changes.
     */
    public RigidBody(double mass, double inertia) {
        if (mass <= 0 || inertia <= 0) {
            throw new IllegalArgumentException("[RigidBody] The values must be greater than 0!");
        }

        inverseMass = 1/mass;
        inverseInertia = 1/inertia;

        oldInverseMass = inverseMass;
        oldInverseInertia = inverseInertia;

        worldPosition = new Vector2D();
        velocity = new Vector2D();
        force = new Vector2D();

        angle = 0;
        angularVelocity = 0;
        torque = 0;
    }

    public Vector2D getWorldPosition() {
        return worldPosition.copy();
    }

    public Vector2D getVelocity() {
        return velocity.copy();
    }

    public Vector2D getForce() {
        return force.copy();
    }

    public double getAngle() {
        return angle;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getTorque() {
        return torque;
    }

    public double getInverseMass() {
        return inverseMass;
    }

    public double getInverseInertia() {
        return inverseInertia;
    }

    public void applyForce(Vector2D force) {
        this.force.add(force);
    }

    public void applyTorque(double torque) {
        this.torque += torque;
    }

    /**
     * Sets infinite mass and inertia to the body,
     * making it unmovable. It also stores the old
     * mass, inertia, in case you need to restore it
     * later.
     */
    public void makeUnmovable() {
        if (inverseMass != 0) {
            oldInverseMass = inverseMass;
        }

        if (inverseInertia != 0) {
            oldInverseInertia = inverseInertia;
        }

        inverseMass = 0;
        inverseInertia = 0;
    }

    /**
     * To be used for restoring mass and inertia after setting
     * infinite mass with makeUnmovable().
     */
    public void resetMovable() {
        inverseMass = oldInverseMass;
        if (rotationLock) return;
        inverseInertia = oldInverseInertia;
    }

    /**
     * Global setter to lock rotation to the axes.
     * It's irreversible.
     */
    public static void lockRotation() {
        rotationLock = true;
    }

    /**
     * Integrates the rigid body with the forces from the world.
     * Calculates acceleration and angular acceleration, and updates
     * the bodies data based on them.
     * Resets the force and torque at the end.
     *
     * @param dt Delta time
     */
    public void integrate(double dt) {
        Vector2D acceleration = getForce().multiply(inverseMass);

        velocity.add(acceleration.multiply(dt));
        worldPosition.add(velocity.multiply(dt));

        if (!rotationLock) {
            double angularAcceleration = torque * inverseInertia;
            angularVelocity += angularAcceleration * dt;
            angle += angularVelocity * dt;
        }

        force = new Vector2D();
        torque = 0;
    }
}