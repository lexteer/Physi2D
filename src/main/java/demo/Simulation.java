package demo;

import physi2d.World;
import physi2d.math.Vector2D;
import physi2d.physics.PhysicsObject;
import physi2d.physics.RigidBody;
import physi2d.shapes.Circle;
import physi2d.shapes.Polygon;
import physi2d.shapes.ShapeType;

import javax.swing.*;
import java.awt.*;

public class Simulation extends JPanel{
    World world;

    static void main(String[] args) {
        new Simulation();
    }

    public Simulation() {
        JFrame frame = new JFrame("Physics engine demo");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(800,600);
        frame.add(this);
        frame.setVisible(true);

        setupPhysicsWorld();

        Timer timer = new Timer(16, e -> {
            int substeps = 8;
            for (int i = 0; i < substeps; i++) {
                world.step(0.016 / substeps);
            }
            repaint();
        });
        timer.start();
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        int scale = 10; // 1 m = 10 px

        // drawing the objects
        for (PhysicsObject object : world.getObjects()) {
            double x = object.getBody().getWorldPosition().x;
            double y = object.getBody().getWorldPosition().y;


            if (object.getShape().getType() == ShapeType.CIRCLE) {
                Circle circle = (Circle) object.getShape();
                double radius = circle.getRadius();

                int screenX = (int) (x * scale);
                int screenY = getHeight() - (int) (y * scale); // flip y, cause of swing
                int rad = (int) (radius * scale);

                g.fillOval(screenX - rad, screenY - rad, rad*2, rad*2);
            } else {
                Polygon rectangle = (Polygon) object.getShape();
                Vector2D[] vertices = rectangle.getVertices();

                int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE;
                int maxX = Integer.MIN_VALUE, maxY = Integer.MIN_VALUE;

                for (Vector2D v : vertices) {
                    int sx = (int)((x + v.x) * scale);
                    int sy = getHeight() - (int)((y + v.y) * scale);
                    minX = Math.min(minX, sx);
                    minY = Math.min(minY, sy);
                    maxX = Math.max(maxX, sx);
                    maxY = Math.max(maxY, sy);
                }

                g.fillRect(minX, minY, maxX - minX, maxY - minY);
            }
        }
    }

    private void setupPhysicsWorld() {
        world = new World();

        // balls
        PhysicsObject ball1 = makeBall(10, 10, 1);
        world.addObject(ball1);
        PhysicsObject ball2 = makeBall(10, 15, 1);
        world.addObject(ball2);
        PhysicsObject ball3 = makeBall(9, 20, 1);
        world.addObject(ball3);
        PhysicsObject ball4 = makeBall(13, 25, 1);
        world.addObject(ball4);

        // floor
        PhysicsObject floor = makeStaticCube(0, 0, 50, 5);
        world.addObject(floor);
    }

    private PhysicsObject makeBall(double x, double y, double radius) {
        Circle circleShape = new Circle(radius);
        RigidBody body = new RigidBody(x, y, 1, 10);
        return new PhysicsObject(body, circleShape);
    }

    private PhysicsObject makeStaticCube(double x, double y, double width, double height) {
        Vector2D[] vertices = {
                new Vector2D(-(width/2), -(height/2)),
                new Vector2D(-(width/2), height/2),
                new Vector2D(width/2, height/2),
                new Vector2D(width/2, -(height/2))
        };
        Polygon rectangle = new Polygon(vertices);
        RigidBody body = new RigidBody(x, y, 10, 10);
        body.makeUnmovable();

        return new PhysicsObject(body, rectangle);
    }
}
