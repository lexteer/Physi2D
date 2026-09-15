package physi2d.math;

import java.util.List;

public record Projection(double min, double max) {
    public double overlapAmount(Projection other) {
        return Math.min(this.max - other.min, other.max - this.min);
    }

    public static Projection project(List<Vec2> worldVertices, Vec2 axis) {
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;

        for (Vec2 vertex : worldVertices) {
            double distanceAlongAxis = vertex.dot(axis);
            min = Math.min(min, distanceAlongAxis);
            max = Math.max(max, distanceAlongAxis);
        }

        return new Projection(min, max);
    }
}
