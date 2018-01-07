package ftc.vision;

import org.opencv.core.Rect;

import java.util.ArrayList;

/**
 * Created by ftc-2875 on 11/12/17.
 */

public class CryptoBoxResult
{


    private int numColumns;
    private int leftColx;
    private int middleColx;
    private int rightColx;

    public CryptoBoxResult(ArrayList<Rect> boxes)
    {
        this.numColumns = boxes.size();
        if (boxes.size()>= 4)
        {
            this.leftColx = boxes.get(0).x;
            this.middleColx = boxes.get(1).x;
            this.rightColx = boxes.get(2).x;

        }


    }

    public int getLeftx()
    {
        return leftColx;
    }
    public int getRightx()
    {
        return rightColx;
    }
    public int getMiddlex()
    {
        return middleColx;
    }
    public int getNumColumns() { return numColumns; }



}
