package ftc.vision;

/**
 * Created by ftc-2875 on 9/17/17.
 */

public class BallCenterResult {
    private int xCoord;
    private int yCoord;
    private boolean foundResult;
    private double area;

    public BallCenterResult(int xCoord, int yCoord, double area) {
        this.xCoord = xCoord;
        this.yCoord = yCoord;
        this.foundResult = true;
        this.area = area;
    }

    public BallCenterResult() {
        this.foundResult = false;
    }

    public int getxCoord() {
        return xCoord;
    }

    public int getyCoord() {
        return yCoord;
    }

    public boolean isFoundResult() {
        return foundResult;
    }

    public double getArea() {
        return area;
    }

    @Override
    public String toString() {
        if (foundResult)
            return "X: " + xCoord + " Y: " + yCoord + "\n Area: " + area;
        else
            return "Nothing found";
    }
}
