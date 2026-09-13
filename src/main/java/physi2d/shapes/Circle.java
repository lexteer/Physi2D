package physi2d.shapes;

public class Circle implements Shape2d {
    private double radius;

    public Circle(double radius) {
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
}
