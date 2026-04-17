package physi2d.physics;

import physi2d.math.Vector2D;

public class CollisionResolution {
    /** Updates the position of the 2 bodies when they collide **/
    public static void resolvePosition(PhysicsObject objectA, PhysicsObject objectB, CollisionManifold manifold) {
        if (!manifold.isColliding()) return;

        double colDepth = manifold.getDepth();
        Vector2D colNormal = manifold.getNormal();

        RigidBody bodyA = objectA.getBody();
        RigidBody bodyB = objectB.getBody();
        double iMassA = bodyA.getInverseMass();
        double iMassB = bodyB.getInverseMass();

        if (iMassA == 0 && iMassB == 0) return;

        double correctionA = colDepth * (iMassA / (iMassA + iMassB));
        double correctionB = colDepth * (iMassB / (iMassA + iMassB));

        Vector2D newPosA = bodyA.getWorldPosition().subtract(colNormal.copy().multiply(correctionA));
        Vector2D newPosB = bodyB.getWorldPosition().add(colNormal.copy().multiply(correctionB));
        bodyA.setWorldPosition(newPosA);
        bodyB.setWorldPosition(newPosB);
    }

    public static void resolveVelocity(PhysicsObject objectA, PhysicsObject objectB, CollisionManifold manifold) {
        if (!manifold.isColliding()) return;

        double e = 0.5; // restitution hardcoded for now
        RigidBody bodyA = objectA.getBody();
        RigidBody bodyB = objectB.getBody();
        double iMassA = bodyA.getInverseMass();
        double iMassB = bodyB.getInverseMass();
        Vector2D velocityA = bodyA.getVelocity();
        Vector2D velocityB = bodyB.getVelocity();

        if (iMassA == 0 && iMassB == 0) return;

        Vector2D relativeVelocity = velocityA.copy().subtract(velocityB);
        double relativeVelocityAlongNormal = relativeVelocity.dotProduct(manifold.getNormal());

        if (relativeVelocityAlongNormal > 0) return;

        double J = -(1 + e) * relativeVelocityAlongNormal / (iMassA + iMassB);

        Vector2D velocityChangeA = manifold.getNormal().copy().multiply(J).multiply(iMassA);
        Vector2D velocityChangeB = manifold.getNormal().copy().multiply(J).multiply(iMassB);

        bodyA.setVelocity(velocityA.copy().subtract(velocityChangeA));
        bodyB.setVelocity(velocityB.copy().add(velocityChangeB));
    }
}
