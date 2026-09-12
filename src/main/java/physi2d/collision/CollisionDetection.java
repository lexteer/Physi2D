package physi2d.collision;

import physi2d.core.Body;
import physi2d.math.MathUtils;
import physi2d.math.Vec2;
import physi2d.shapes.Circle;

import java.util.List;
import java.util.Optional;

public class CollisionDetection {
    public static Optional<CollisionManifold> circleCircle(Body bodyA, Body bodyB) {
        Circle circleA = (Circle) bodyA.getShape();
        Circle circleB = (Circle) bodyB.getShape();
        double radiiSum = circleA.getRadius() + circleB.getRadius();

        Vec2 vecFromAtoB = bodyB.getPosition().sub(bodyA.getPosition());
        double distanceSquared = vecFromAtoB.lengthSquared();
        if (distanceSquared >= radiiSum * radiiSum) return Optional.empty(); // not colliding

        double distance = vecFromAtoB.length();
        if (distance < MathUtils.EPSILON) {
            Vec2 normal = new Vec2(1,0);
            return Optional.of(new CollisionManifold(bodyA, bodyB, normal, radiiSum, List.of(bodyA.getPosition())));
        }

        Vec2 normal = vecFromAtoB.mult(1/distance);
        double depth = radiiSum - distance;
        Vec2 contactPoint = bodyA.getPosition().add(normal.mult(circleA.getRadius() - depth/2));

        return Optional.of(new CollisionManifold(bodyA, bodyB, normal, depth, List.of(contactPoint)));
    }
}
