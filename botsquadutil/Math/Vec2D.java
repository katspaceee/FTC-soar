package org.firstinspires.ftc.teamcode.botsquadutil.Math;

public class Vec2D {
    public double x, y;

    public Vec2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void add(Vec2D vec) {
        this.x += vec.x;
        this.y += vec.y;
    }

    public void subtract(Vec2D vec) {
        this.x -= vec.x;
        this.y -= vec.y;
    }

    static public Vec2D add(Vec2D vec1, Vec2D vec2) {
        return new Vec2D(vec1.x + vec2.x, vec1.y + vec2.y);
    }

    static public Vec2D subtract(Vec2D vec1, Vec2D vec2) {
        return new Vec2D(vec1.x - vec2.x, vec1.y - vec2.y);
    }

    static public Vec2D dot(Vec2D vec1, Vec2D vec2) {
        return new Vec2D(vec1.x * vec2.x, vec1.y * vec2.y);
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }
}

