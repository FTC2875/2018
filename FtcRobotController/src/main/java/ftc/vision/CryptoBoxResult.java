package ftc.vision;

/**
 * Created by ftc-2875 on 11/12/17.
 */

public class CryptoBoxResult
{


    private int numColumns;
    private int leftColx;
    private int middleColx;
    private int rightColx;

    public CryptoBoxResult(int numColumns, int leftColx, int middleColx, int rightColx)
    {
        this.numColumns = numColumns;
        this.leftColx = leftColx;
        this.middleColx = middleColx;
        this.rightColx = rightColx;

    }

    public int getLeftx()
    {
        return leftColx;
    }

    public int getRightx()
    {
        return rightColx;
    }
    public int getMiddleColx()
    {
        return middleColx;
    }
    public int getNumColumns()
    {
        return numColumns;
    }


}
