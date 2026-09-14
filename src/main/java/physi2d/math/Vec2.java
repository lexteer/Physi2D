package physi2d.math;

public record Vec2(double x, double y) {
    public static final Vec2 ZERO = new Vec2(0,0);

    @Override
    public String toString() {
        return "[" + x + "," + y + "]";
    }

    public Vec2 add(Vec2 other) {
        return new Vec2(this.x + other.x, this.y + other.y);
    }

    public Vec2 sub(Vec2 other) {
        return new Vec2(this.x - other.x, this.y - other.y);
    }

    public Vec2 mult(double scalar) {
        return new Vec2(this.x * scalar, this.y * scalar);
    }

    public Vec2 negate() {
        return mult(-1);
    }

    public double dot(Vec2 other) {
        return this.x * other.x + this.y * other.y;
    }

    public double cross(Vec2 other) {
        return this.x * other.y - this.y * other.x;
    }

    public double lengthSquared() {
        return this.x * this.x + this.y * this.y;
    }

    public double length() {
        return Math.sqrt(lengthSquared());
    }

    public Vec2 normalize() {
        double len = length();
        if (len < MathUtils.EPSILON) throw new ArithmeticException("Cannot normalize a vector with a length of 0");
        return new Vec2(this.x / len, this.y / len);
    }

    public Vec2 rotate(double angle) {
        double cosAngle = Math.cos(angle);
        double sinAngle = Math.sin(angle);

        return rotate(cosAngle, sinAngle);
    }

    public Vec2 rotate(double cosAngle, double sinAngle) {
        double rotatedX = this.x * cosAngle - this.y * sinAngle;
        double rotatedY = this.x * sinAngle + this.y * cosAngle;
        return new Vec2(rotatedX, rotatedY);
    }
}
