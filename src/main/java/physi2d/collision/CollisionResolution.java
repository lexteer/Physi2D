package physi2d.collision;

import physi2d.core.Body;
import physi2d.math.Vec2;

public class CollisionResolution {
    private static final double SLOP = 0.005;
    private static final double CORRECTION_PERCENTAGE = 0.2;

    public static void resolve(CollisionManifold manifold) {
        Body bodyA = manifold.bodyA();
        Body bodyB = manifold.bodyB();
        double iMassA = bodyA.getInvMass();
        double iMassB = bodyB.getInvMass();
        double iMassSum = iMassA + iMassB;
        if (iMassSum == 0) return;

        Vec2 relVel = bodyB.getVelocity().sub(bodyA.getVelocity());
        Vec2 normal = manifold.normal();
        double relVelDotNor = relVel.dot(normal);
        if (relVelDotNor > 0.0) return;

        double e = Math.min(bodyA.getRestitution(), bodyB.getRestitution());

        double j = (-(1.0 + e) * relVelDotNor) / iMassSum;

        Vec2 impulse = normal.mult(j);

        Vec2 aVelocity = bodyA.getVelocity().sub(impulse.mult(iMassA));
        Vec2 bVelocity = bodyB.getVelocity().add(impulse.mult(iMassB));

        bodyA.setVelocity(aVelocity);
        bodyB.setVelocity(bVelocity);

        positionalCorrection(manifold);
    }

    private static void applyContactFriction() {

    }

    private static void positionalCorrection(CollisionManifold manifold) {
        Body bodyA = manifold.bodyA();
        Body bodyB = manifold.bodyB();
        double iMassA = bodyA.getInvMass();
        double iMassB = bodyB.getInvMass();
        double iMassSum = iMassA + iMassB;

        double amountToCorrect = (Math.max(manifold.depth() - SLOP, 0.0) / iMassSum) * CORRECTION_PERCENTAGE;
        Vec2 correctionVector = manifold.normal().mult(amountToCorrect);

        Vec2 positionA = bodyA.getPosition();
        Vec2 positionB = bodyB.getPosition();
        bodyA.setPosition(positionA.sub(correctionVector.mult(iMassA)));
        bodyB.setPosition(positionB.add(correctionVector.mult(iMassB)));
    }

}
