package physi2d.shapes;

public class CircleShape implements Shape2d {
    private double radius;

    public CircleShape(double radius) {
        this.radius = radius;
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    @Override
    public double computeInertia(double mass) {
        return 0.5 * mass * radius * radius;
    }

    @Override
    public ShapeType getType() {
        return ShapeType.CIRCLE;
    }
}
