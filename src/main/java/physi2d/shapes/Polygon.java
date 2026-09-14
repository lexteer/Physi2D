package physi2d.shapes;

import physi2d.math.MathUtils;
import physi2d.math.Vec2;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Polygon implements Shape2d {
    private List<Vec2> vertices;
    private double area;

    public Polygon(List<Vec2> vertices) {
        this.vertices = new ArrayList<>(vertices);
        centerPolygonOnCentroid(computeCentroid());
    }

    public List<Vec2> getVertices() {
        return vertices;
    }

    public List<Vec2> getWorldVertices(double angle, Vec2 position) {
        List<Vec2> worldVertices = new ArrayList<>();
        double cosAngle = Math.cos(angle);
        double sinAngle = Math.sin(angle);

        for (Vec2 vertex : vertices) {
            worldVertices.add(vertex.rotate(cosAngle, sinAngle).add(position));
        }

        return worldVertices;
    }

    private void centerPolygonOnCentroid(Vec2 centroid) {
        List<Vec2> newVertices = new ArrayList<>();
        for (Vec2 vertex : vertices) {
            newVertices.add(vertex.sub(centroid));
        }
        vertices = Collections.unmodifiableList(newVertices);
    }

    private Vec2 computeCentroid() {
        int n = vertices.size();
        double areaSum = 0;
        double centroidXSum = 0;
        double centroidYSum = 0;

        for (int i = 0; i < n; i++) {
            Vec2 current = vertices.get(i);
            Vec2 next = vertices.get((i + 1) % n);

            double crossTerm = current.cross(next);
            areaSum += crossTerm;

            centroidXSum += (current.x() + next.x()) * crossTerm;
            centroidYSum += (current.y() + next.y()) * crossTerm;
        }

        double signedArea = areaSum / 2.0;
        area = Math.abs(signedArea);

        if (area < MathUtils.EPSILON) {
            throw new IllegalArgumentException("Cannot build a polygon; all vertices are on a line!");
        }

        if (signedArea < 0) {
            Collections.reverse(vertices);
            signedArea *= -1;
            centroidXSum *= -1;
            centroidYSum *= -1;
            System.err.println("Polygon should have vertices in counter clock wise order!");
        }

        double centroidX = centroidXSum / (6.0 * signedArea);
        double centroidY = centroidYSum / (6.0 * signedArea);

        return new Vec2(centroidX, centroidY);
    }

    @Override
    public double computeInertia(double mass) {
        int n = vertices.size();
        double inertiaSum = 0;

        for (int i = 0; i < n; i++) {
            Vec2 current = vertices.get(i);
            Vec2 next = vertices.get((i + 1) % n);

            double crossTerm = current.cross(next);
            double dotCurrentCurrent = current.dot(current);
            double dotCurrentNext = current.dot(next);
            double dotNextNext = next.dot(next);
            double dotSum = dotCurrentCurrent + dotCurrentNext + dotNextNext;

            inertiaSum += crossTerm * dotSum;
        }

        return (mass * inertiaSum) / (12.0 * area);
    }
}
