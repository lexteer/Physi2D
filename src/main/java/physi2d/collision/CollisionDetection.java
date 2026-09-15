package physi2d.collision;

import physi2d.core.Body;
import physi2d.math.MathUtils;
import physi2d.math.Projection;
import physi2d.math.Vec2;
import physi2d.shapes.Circle;
import physi2d.shapes.Polygon;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CollisionDetection {
    private record SatResult(Vec2 axis, double overlap, int faceIndex) {}

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

    public static Optional<CollisionManifold> polyPoly(Body bodyA, Body bodyB) {
        Polygon polyA = (Polygon) bodyA.getShape();
        Polygon polyB = (Polygon) bodyB.getShape();

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

    private static Optional<SatResult> doSATonPolygons(List<Vec2> verticesA, List<Vec2> verticesB) {
        double smallestOverlap = Double.POSITIVE_INFINITY;
        Vec2 axis = Vec2.ZERO;
        int index = 0;

        for (int i = 0; i < verticesA.size(); i++) {
            Vec2 faceNormal = Polygon.getFaceNormal(verticesA, i);
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
            Vec2 edgeNormal = Polygon.getFaceNormal(incidentPoly, i);
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
