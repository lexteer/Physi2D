package physi2d.physics;

import physi2d.math.Vector2D;

public class CollisionResolution {
    public static void resolvePosition(PhysicsObject objectA, PhysicsObject objectB, CollisionManifold manifold) {
        if (!manifold.isColliding()) return;

        double colDepth = manifold.getDepth();
        Vector2D colNormal = manifold.getNormal();

        RigidBody bodyA = objectA.getBody();
        RigidBody bodyB = objectB.getBody();
        double iMassA = bodyA.getInverseMass();
        double iMassB = bodyB.getInverseMass();

        if (iMassA == 0 && iMassB == 0) {
            return;
        }

        double correctionA = colDepth * (iMassA / (iMassA + iMassB));
        double correctionB = colDepth * (iMassB / (iMassA + iMassB));

        Vector2D newPosA = bodyA.getWorldPosition().subtract(colNormal.copy().multiply(correctionA));
        Vector2D newPosB = bodyB.getWorldPosition().add(colNormal.copy().multiply(correctionB));
        bodyA.setWorldPosition(newPosA);
        bodyB.setWorldPosition(newPosB);
    }
}
