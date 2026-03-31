package physics;

import shapes.Shape;

/**
 * Pure data class holding a rigid body and its shape
 */
public class PhysicsObject {
    private RigidBody body;
    private Shape shape;

    public PhysicsObject(RigidBody body, Shape shape) {
        this.body = body;
        this.shape = shape;
    }

    public RigidBody getBody() {
        return body;
    }

    public Shape getShape() {
        return shape;
    }
}
