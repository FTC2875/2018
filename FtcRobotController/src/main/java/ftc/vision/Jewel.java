package ftc.vision;

/**
 *

 * Created by ftc-2875 on 12/3/17.
 */

public class Jewel {
    private double area;
    private BallColor color;
    private int centerX;
    private int centerY;

    public Jewel(double area, BallColor color, int centerX, int centerY) {
        this.area = area;
        this.color = color;
        this.centerX = centerX;
        this.centerY = centerY;
    }

    public double getArea() {
        return area;
    }

    public BallColor getColor() {
        return color;
    }

    public int getCenterX() {
        return centerX;
    }

    public int getCenterY() {
        return centerY;
    }

    public String toString() {
        return color.toString();
    }
}


