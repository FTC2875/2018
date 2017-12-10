package ftc.vision;

/**
 * Created by ftc-2875 on 9/17/17.
 */

public class BallCenterResult {
    private Jewel leftJewel;
    private Jewel rightJewel;
    private boolean foundResult;

    private final int BLUE_CENTER_X = 390;

    public BallCenterResult(Jewel redJewel, Jewel blueJewel) {
        this.foundResult = true;

        if (blueJewel.getCenterX() < BLUE_CENTER_X) {
            leftJewel = blueJewel;
            rightJewel = redJewel;
        } else {
            leftJewel = redJewel;
            rightJewel = blueJewel;
        }

        // debug
//        leftJewel = blueJewel;
//        rightJewel = redJewel;

    }

    public Jewel getLeftJewel() {
        return leftJewel;
    }

    public Jewel getRightJewel() {
        return rightJewel;
    }

    public BallCenterResult() {
        this.foundResult = false;
    }


    public boolean isFoundResult() {
        return foundResult;
    }


    @Override
    public String toString() {
        if (foundResult)
            return "Found";
        else
            return "Nothing found";
    }
}
