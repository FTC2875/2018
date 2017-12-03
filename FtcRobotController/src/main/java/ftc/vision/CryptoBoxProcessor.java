package ftc.vision;

import android.provider.ContactsContract;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Created by ftc-2875 on 11/12/17.
 */

public class CryptoBoxProcessor implements ImageProcessor<CryptoBoxResult>{
    
    public static final String TAG = "CryptoBox";

    private final Scalar LOWER_BLUE = new Scalar(90, 135, 25);
    private final Scalar UPPER_BLUE = new Scalar(130, 250, 150);

    private Rect myRect = new Rect();

    private ArrayList<Rect> boxes;
    private ArrayList<Rect> arrangedBoxes;

    @Override
    public ImageProcessorResult<CryptoBoxResult> process(long startTime, Mat rgbaFrame, boolean saveImages) {
        Mat hsv = rgbaFrame.clone();
        Mat erode = hsv.clone();
        Mat dilate = hsv.clone();
        Mat blur = hsv.clone();

        Mat kernel = new Mat(5, 5, CvType.CV_8U);

        Imgproc.cvtColor(rgbaFrame, hsv, Imgproc.COLOR_RGB2HSV);

        // get rid of noise
        Imgproc.erode(hsv, erode, kernel);
        Imgproc.dilate(erode, dilate, kernel);
        Imgproc.blur(dilate, blur, new Size(6, 6));

        Mat thresh = blur.clone();
        Core.inRange(blur, LOWER_BLUE, UPPER_BLUE, thresh);
        Log.d(TAG, "process: finished thresholding in range");

        Mat struct = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 30));
        Log.d(TAG, "process: finished getting struct");
        Log.d(TAG, "process: making clone of threshold");
        Mat morphedThresh = thresh.clone();
        Imgproc.morphologyEx(thresh, morphedThresh, Imgproc.MORPH_CLOSE, struct);
        Log.d(TAG, "process: finished morphing thresh");

        Mat uselessH = morphedThresh.clone();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morphedThresh, contours, uselessH, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Log.d(TAG, "process: finished finding contours");


        boxes = new ArrayList<>();


        for (int i =0; i< contours.size(); i++)
        {
            if (Imgproc.contourArea(contours.get(i)) >= 100)
            {
                myRect = Imgproc.boundingRect(contours.get(i));

                double ratio = Math.abs(myRect.height/myRect.width);

                if (ratio > 1.5)
                {
                    boxes.add(myRect);
                }
 // haaha
            }

        }
        arrangedBoxes = new ArrayList<>();

            int n = boxes.size();

            // One by one move boundary of unsorted subarray
            for (int i = 0; i < n-1; i++)
            {
                // Find the minimum element in unsorted array
                int min_idx = i;
                for (int j = i+1; j < n; j++)
                    if (boxes.get(j).x < boxes.get(min_idx).x)
                        min_idx = j;

                // Swap the found minimum element with the first
                // element

                Rect tbox = boxes.get(min_idx);
                boxes.set(min_idx, boxes.get(i));
                boxes.set(i, tbox);


            }




        for (int a = 0; a < boxes.size(); a++)
        {
            Rect currentBox = boxes.get(a);
            Imgproc.rectangle(rgbaFrame, new Point(currentBox.x , currentBox.y), new Point(currentBox.x + currentBox.width, currentBox.y + currentBox.height), new Scalar(255,0,0));
        }
        // release to free up memory
        hsv.release();
        erode.release();
        dilate.release();
        thresh.release();
        struct.release();
        Log.d(TAG, "process: finished freeing up memory");

        return new ImageProcessorResult<>(startTime, rgbaFrame,  new CryptoBoxResult(boxes.size(), boxes.get(0).x, boxes.get(1).x, boxes.get(2).x));


    }




    }




