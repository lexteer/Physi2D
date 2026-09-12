package physi2d.shapes;

public class Circle implements Shape2d{
    private double radius;

    public Circle(double radius) {
        this.radius = radius;
    }

    public double getRadius() {
        return radius;
    }
}
