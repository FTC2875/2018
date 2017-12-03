package ftc.vision;

/**
 * Created by ftc-2875 on 9/17/17.
 */

public class BallCenterResult {
    private Jewel leftJewel;
    private Jewel rightJewel;
    private boolean foundResult;

    private final int BLUE_CENTER_X = 300;

    public BallCenterResult(Jewel redJewel, Jewel blueJewel) {
        this.foundResult = true;

        // TODO get rid of this
        if (redJewel.getCenterX() < blueJewel.getCenterX()) {
            leftJewel = redJewel;
            rightJewel = blueJewel;
        } else {
            leftJewel = blueJewel;
            rightJewel = redJewel;
        }

        // todo replace with this
//        if (blueJewel.getCenterX() < BLUE_CENTER_X) {
//            leftJewel = blueJewel;
//            rightJewel = redJewel;
//        } else {
//            leftJewel = redJewel;
//            rightJewel = blueJewel;
//        }

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
