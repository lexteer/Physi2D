package com.github.lexteer.physics;

import math.Vector2D;

/**
 * Data class that holds the result of a narrow phase collision check.
 * Used for collision resolution.
 */
public class CollisionManifold {
    private boolean hasCollision;
    private Vector2D normal;
    private double depth;

    public CollisionManifold(boolean hasCollision, Vector2D normal, double depth) {
        this.hasCollision = hasCollision;
        this.normal = normal;
        this.depth = depth;
    }

    public boolean isColliding() {
        return hasCollision;
    }

    public Vector2D getNormal() {
        return normal;
    }

    public double getDepth() {
        return depth;
    }
}
