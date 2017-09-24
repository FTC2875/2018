package ftc.vision;

/**
 * Created by ftc-2875 on 9/17/17.
 */

public class BallCenterResult {
    private int xCoord;
    private int yCoord;
    private boolean foundResult;

    public BallCenterResult(int xCoord, int yCoord) {
        this.xCoord = xCoord;
        this.yCoord = yCoord;
        this.foundResult = true;
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

    @Override
    public String toString() {
        return "X: " + xCoord + " Y: " + yCoord;
    }
}
