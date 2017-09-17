package ftc.vision;

/**
 * Created by ftc-2875 on 9/17/17.
 */

public class BallCenterResult {
    private int xCoord;
    private int yCoord;

    public BallCenterResult(int xCoord, int yCoord) {
        this.xCoord = xCoord;
        this.yCoord = yCoord;
    }

    public int getxCoord() {
        return xCoord;
    }

    public int getyCoord() {
        return yCoord;
    }

    @Override
    public String toString() {
        return "X: " + xCoord + "Y: " + yCoord;
    }
}
