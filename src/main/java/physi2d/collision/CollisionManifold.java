package physi2d.collision;

import physi2d.core.Body;
import physi2d.math.Vec2;

import java.util.List;

public record CollisionManifold(Body bodyA, Body bodyB, Vec2 normal, double depth, List<Vec2> contactPoints) {}
