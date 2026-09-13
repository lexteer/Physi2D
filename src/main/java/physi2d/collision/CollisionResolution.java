package physi2d.collision;

import physi2d.core.Body;
import physi2d.math.MathUtils;
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
        applyImpulseToVelocity(bodyA, bodyB, impulse);

        applyContactFriction(manifold, relVel, j);
        positionalCorrection(manifold);
    }

    private static void applyContactFriction(CollisionManifold manifold, Vec2 relVel, double j) {
        Body bodyA = manifold.bodyA();
        Body bodyB = manifold.bodyB();
        double iMassA = bodyA.getInvMass();
        double iMassB = bodyB.getInvMass();
        double iMassSum = iMassA + iMassB;

        Vec2 normal = manifold.normal();
        double normalComponent = relVel.dot(normal);
        Vec2 projection = normal.mult(normalComponent);

        Vec2 tangent = relVel.sub(projection);
        if (tangent.lengthSquared() < MathUtils.EPSILON) return;
        tangent = tangent.normalize();

        double jt = -(relVel.dot(tangent)) / iMassSum;

        Vec2 frictionImpulse;
        double staticFriction = Math.sqrt(bodyA.getStaticFriction() * bodyB.getStaticFriction());
        double dynamicFriction = Math.sqrt(bodyA.getDynamicFriction() * bodyB.getDynamicFriction());

        if (Math.abs(jt) <= j * staticFriction) {
            frictionImpulse = tangent.mult(jt);
        } else {
            frictionImpulse = tangent.mult(-dynamicFriction * j);
        }

        applyImpulseToVelocity(bodyA, bodyB, frictionImpulse);
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

    private static void applyImpulseToVelocity(Body bodyA, Body bodyB, Vec2 impulse) {
        double iMassA = bodyA.getInvMass();
        double iMassB = bodyB.getInvMass();

        Vec2 aVelocity = bodyA.getVelocity().sub(impulse.mult(iMassA));
        Vec2 bVelocity = bodyB.getVelocity().add(impulse.mult(iMassB));

        bodyA.setVelocity(aVelocity);
        bodyB.setVelocity(bVelocity);
    }
}
