package physi2d.shapes;

/**
 * To be used to track min and max values of a projection
 * and check if it is overlapping with other projections.
 *
 * @param min Min value of a projection on an axis.
 * @param max Max value of a projection on an axis.
 */
public record Projection(double min, double max) {
    /**
     * Checks if two projections are overlapping.
     *
     * @param other Projection with which it is checking
     * @return True if the two projections are overlapping
     */
    public boolean overlaps(Projection other) {
        return max > other.min && other.max > this.min;
    }

    /**
     * If the two projections are overlapping, this checks
     * by how much aka the overlap depth.
     *
     * @param other Projection with which it is checking
     * @return Overlap depth of the projection
     */
    public double getOverlapDepth(Projection other) {
        if (!overlaps(other)) return 0;

        double depthToRight = this.max - other.min;
        double depthToLeft = other.max - this.min;

        return Math.min(depthToRight, depthToLeft);
    }
}