package physi2d.math;

/**
 * A MUTABLE 2D Vector class that can be used for holding data about
 * vectors and points in 2d space. There are also methods for manipulating
 * the data.
 * Many methods like add and subtract directly change the data, so be carefull!
 */
public class Vector2D {
    public double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D() {
        x = 0;
        y = 0;
    }

    @Override
    public String toString() {
        return "X: " + x + " Y: " + y;
    }

    public Vector2D add(Vector2D vector2D) {
        x += vector2D.x;
        y += vector2D.y;
        return this;
    }

    public Vector2D subtract(Vector2D vector2D) {
        x -= vector2D.x;
        y -= vector2D.y;
        return this;
    }

    public Vector2D multiply(double value) {
        x *= value;
        y *= value;
        return this;
    }

    public Vector2D copy() {
        return new Vector2D(this.x, this.y);
    }

    // aka the length
    public double magnitude() {
        return Math.sqrt(x*x + y*y);
    }

    public Vector2D normalize() {
        double magnitude = magnitude();
        if (magnitude == 0) throw new ArithmeticException("[Vector2D] Vector cannot be normalized, because of a magnitude of 0.");
        x /= magnitude;
        y /= magnitude;
        return this;
    }

    public double dotProduct(Vector2D other) {
        return (this.x * other.x) + (this.y * other.y);
    }

    public double distance(Vector2D other) {
        Vector2D difference = new Vector2D(other.x - this.x, other.y - this.y);

        return difference.magnitude();
    }

    /**
     * @param rad Radians, not degrees
     * @return the mutated instance
     */
    public Vector2D rotate(double rad) {
        double xCos = x * Math.cos(rad);
        double yCos = y * Math.cos(rad);
        double xSin = x * Math.sin(rad);
        double ySin = y * Math.sin(rad);

        x = xCos - ySin;
        y = xSin + yCos;

        return this;
    }
}
