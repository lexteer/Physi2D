package physi2d.collision;

import physi2d.core.Body;
import physi2d.math.MathUtils;
import physi2d.math.Vec2;
import physi2d.shapes.CircleShape;
import physi2d.shapes.PolygonShape;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CollisionDetection {
    private record SatResult(Vec2 axis, double overlap, int faceIndex) {}

    public static Optional<CollisionManifold> circleCircle(Body bodyA, Body bodyB) {
        CircleShape circleA = (CircleShape) bodyA.getShape();
        CircleShape circleB = (CircleShape) bodyB.getShape();
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

    public static Optional<CollisionManifold> circlePoly(Body circleBody, Body polyBody) {
        PolygonShape poly = (PolygonShape) polyBody.getShape();
        List<Vec2> wVertices = poly.getWorldVertices(polyBody.getAngle(), polyBody.getPosition());

        Optional<SatResult> faceTestOpt = doSATonCircle(wVertices, circleBody);
        if (faceTestOpt.isEmpty()) return Optional.empty();
        SatResult faceTest = faceTestOpt.get();

        Vec2 closestCornerAxis = getClosestCornerAxisToCircle(wVertices, circleBody);

        double cornerOverlap = circleToPolyOverlap(wVertices, circleBody, closestCornerAxis);
        if (cornerOverlap <= 0) return Optional.empty();

        double depth;
        Vec2 normal;

        if (cornerOverlap < faceTest.overlap()) {
            depth = cornerOverlap;
            normal = closestCornerAxis;
        } else {
            depth = faceTest.overlap();
            normal = faceTest.axis();
        }

        normal = normal.negate();

        // contact point
        CircleShape circle = (CircleShape) circleBody.getShape();
        Vec2 circleCenter = circleBody.getPosition();
        double radius = circle.getRadius();

        Vec2 contactPoint = normal.mult(radius).add(circleCenter);

        return Optional.of(new CollisionManifold(circleBody, polyBody, normal, depth, List.of(contactPoint)));
    }

    public static Optional<CollisionManifold> polyPoly(Body bodyA, Body bodyB) {
        PolygonShape polyA = (PolygonShape) bodyA.getShape();
        PolygonShape polyB = (PolygonShape) bodyB.getShape();

        List<Vec2> wVerticesA = polyA.getWorldVertices(bodyA.getAngle(), bodyA.getPosition());
        List<Vec2> wVerticesB = polyB.getWorldVertices(bodyB.getAngle(), bodyB.getPosition());

        Optional<SatResult> resultA = doSATonPolygons(wVerticesA, wVerticesB);
        if (resultA.isEmpty()) return Optional.empty();

        Optional<SatResult> resultB = doSATonPolygons(wVerticesB, wVerticesA);
        if (resultB.isEmpty()) return Optional.empty();

        SatResult result = (resultA.get().overlap() < resultB.get().overlap()) ? resultA.get() : resultB.get();

        Vec2 normal = result.axis();
        if (result == resultB.get()) normal = normal.negate();

        // contact points
        List<Vec2> reference = (result == resultA.get()) ? wVerticesA : wVerticesB;
        List<Vec2> incident = (result == resultA.get()) ? wVerticesB : wVerticesA;
        Vec2 refCorner1 = reference.get(result.faceIndex());
        Vec2 refCorner2 = reference.get((result.faceIndex() + 1) % reference.size());

        int incidentFace = getIncidentFace(incident, result.axis());
        Vec2 incCorner1 = incident.get(incidentFace);
        Vec2 incCorner2 = incident.get((incidentFace + 1) % incident.size());

        Vec2 refEdge = refCorner2.sub(refCorner1);

        List<Vec2> clipped = clip(incCorner1, incCorner2, refCorner1, refEdge);
        if (clipped.size() < 2) return Optional.empty();

        Vec2 oppositeEdge = refEdge.negate();
        clipped = clip(clipped.get(0), clipped.get(1), refCorner2, oppositeEdge);
        if (clipped.size() < 2) return Optional.empty();

        List<Vec2> contactPoints = new ArrayList<>();

        for (Vec2 point : clipped) {
            Vec2 offset = point.sub(refCorner1);
            double separation = offset.dot(result.axis());

            if (separation <= 0) contactPoints.add(point);
        }

        if (contactPoints.isEmpty()) return Optional.empty();

        return Optional.of(new CollisionManifold(bodyA, bodyB, normal, result.overlap(), contactPoints));
    }

    private static double circleToPolyOverlap(List<Vec2> wVertices, Body circleBody, Vec2 closestCornerAxis) {
        CircleShape circle = (CircleShape) circleBody.getShape();
        Vec2 circleCenter = circleBody.getPosition();
        double radius = circle.getRadius();

        double largestDistance = Double.NEGATIVE_INFINITY;
        for (Vec2 vertex : wVertices) {
            double reachDistance = vertex.dot(closestCornerAxis);

            if (reachDistance > largestDistance) largestDistance = reachDistance;
        }

        double circleNearSide = circleCenter.dot(closestCornerAxis) - radius;

        return largestDistance - circleNearSide;
    }


    private static Vec2 getClosestCornerAxisToCircle(List<Vec2> polygonVertices, Body circleBody) {
        Vec2 circleCenter = circleBody.getPosition();
        double smallestSquaredDistance = Double.POSITIVE_INFINITY;
        Vec2 closestVertex = polygonVertices.getFirst();

        for (Vec2 vertex : polygonVertices) {
            Vec2 offset = circleCenter.sub(vertex);
            double distanceSquared = offset.lengthSquared();

            if (distanceSquared < smallestSquaredDistance) {
                smallestSquaredDistance = distanceSquared;
                closestVertex = vertex;
            }
        }

        if (smallestSquaredDistance < MathUtils.EPSILON) {
            return new Vec2(1, 0);
        }

        return circleCenter.sub(closestVertex).normalize();
    }

    private static Optional<SatResult> doSATonCircle(List<Vec2> polygonVertices, Body circleBody) {
        double radius = ((CircleShape) circleBody.getShape()).getRadius();
        Vec2 circleCenter = circleBody.getPosition();
        double smallestOverlap = Double.POSITIVE_INFINITY;
        Vec2 axis = Vec2.ZERO;
        int index = 0;

        for (int i = 0; i < polygonVertices.size(); i++) {
            Vec2 vertex = polygonVertices.get(i);
            Vec2 faceNormal = PolygonShape.getFaceNormal(polygonVertices, i);
            Vec2 offset = circleCenter.sub(vertex);
            double distance = offset.dot(faceNormal) - radius;

            double overlap = -distance;
            if (overlap <= 0) return Optional.empty();

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                axis = faceNormal;
                index = i;
            }
        }

        return Optional.of(new SatResult(axis, smallestOverlap, index));
    }

    private static Optional<SatResult> doSATonPolygons(List<Vec2> verticesA, List<Vec2> verticesB) {
        double smallestOverlap = Double.POSITIVE_INFINITY;
        Vec2 axis = Vec2.ZERO;
        int index = 0;

        for (int i = 0; i < verticesA.size(); i++) {
            Vec2 faceNormal = PolygonShape.getFaceNormal(verticesA, i);
            Vec2 faceVertex = verticesA.get(i);
            double deepest = Double.POSITIVE_INFINITY;

            for (Vec2 vertex : verticesB) {
                double distance = vertex.sub(faceVertex).dot(faceNormal);
                deepest = Math.min(deepest, distance);
            }

            double overlap = -deepest;
            if (overlap <= 0) return Optional.empty();

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                axis = faceNormal;
                index = i;
            }
        }

        return Optional.of(new SatResult(axis, smallestOverlap, index));
    }

    private static int getIncidentFace(List<Vec2> incidentPoly, Vec2 referenceAxis) {
        double smallestEdge = Double.POSITIVE_INFINITY;
        int index = 0;

        for (int i = 0; i < incidentPoly.size(); i++) {
            Vec2 edgeNormal = PolygonShape.getFaceNormal(incidentPoly, i);
            double facing = edgeNormal.dot(referenceAxis);

            if (facing < smallestEdge) {
                smallestEdge = facing;
                index = i;
            }
        }

        return index;
    }

    private static List<Vec2> clip(Vec2 point1, Vec2 point2, Vec2 planePoint, Vec2 planeDirection) {
        List<Vec2> kept = new ArrayList<>();

        Vec2 offset1 = point1.sub(planePoint);
        Vec2 offset2 = point2.sub(planePoint);
        double distance1 = offset1.dot(planeDirection);
        double distance2 = offset2.dot(planeDirection);

        if (distance1 >= 0) kept.add(point1);
        if (distance2 >= 0) kept.add(point2);

        if (distance1 * distance2 < 0) {
            Vec2 segment = point2.sub(point1);
            double t = distance1 / (distance1 - distance2);
            Vec2 intersection = point1.add(segment.mult(t));
            kept.add(intersection);
        }

        return kept;
    }
}
