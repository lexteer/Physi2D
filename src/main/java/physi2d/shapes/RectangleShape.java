package physi2d.shapes;

import physi2d.math.Vec2;

import java.util.List;

public class RectangleShape extends PolygonShape{
    public RectangleShape(double width, double height) {
        if (width <= 0 || height <= 0) throw new IllegalArgumentException("Width and height must be greater than 0");

        double halfW = width/2;
        double halfH = height/2;

        List<Vec2> vertices = List.of(
                new Vec2(-halfW, -halfH),
                new Vec2(halfW, -halfH),
                new Vec2(halfW, halfH),
                new Vec2(-halfW, halfH)
        );

        super(vertices);
    }
}
