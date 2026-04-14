package com.github.lexteer.shapes;

/**
 * Represents an AABB (axis-aligned bounding box) shape used for broad phase
 * collision detection. An AABB is the tightest non-rotated rectangle that
 * fully contains a shape at its current world position and rotation.
 */
public class AABB {
    private double minX, maxX, minY, maxY;

    public AABB(double minX, double maxX, double minY, double maxY) {
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    /**
     * Uses the Separating Axis Theorem (SAT) on the two axes (x, y).
     *
     * @param other The other AABB shape to test this AABB instance against
     * @return true if the two AABB boxes overlap
     */
    public boolean overlaps(AABB other) {
        return this.maxX > other.minX &&
                this.minX < other.maxX &&
                this.maxY > other.minY &&
                this.minY < other.maxY;
    }

    public double getMinX() {
        return minX;
    }

    public double getMaxX() {
        return maxX;
    }

    public double getMinY() {
        return minY;
    }

    public double getMaxY() {
        return maxY;
    }
}
