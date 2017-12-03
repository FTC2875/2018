package ftc.vision;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.Feature2D;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.objdetect.Objdetect;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ftc-2875 on 9/17/17.
 */

public class BallCenterProcessor implements ImageProcessor<BallCenterResult> {

    private final String TAG = "Image Processor";
    private boolean detectRed;

    // lower bounds has the upper value because HSV cylinder
    private Scalar redHsvLower = new Scalar(174, 222, 0);
    private Scalar redHsvUpper = new Scalar(5, 255, 255);

    private Scalar blueHsvLower = new Scalar(99, 250, 0);
    private Scalar blueHsvUpper = new Scalar(113, 255, 255);

//    private final Scalar blueHsvLower = new Scalar(0, 0, 0);
//    private final Scalar blueHsvUpper = new Scalar(179, 255, 255);

    public BallCenterProcessor(boolean detectRed) {
        this.detectRed = detectRed;
    }

    @Override
    public ImageProcessorResult<BallCenterResult> process(long startTime, Mat rgbaFrame, boolean saveImages) {
        Log.i(TAG, "Starting to image process ball");
        Mat redHSV = rgbaFrame.clone();
        Mat blueHSV = rgbaFrame.clone();
        Mat hierachy = rgbaFrame.clone();
        Mat thresholdedHSVBlue = rgbaFrame.clone();
        Mat thresholdedHSVRed = rgbaFrame.clone();

        List<MatOfPoint> contoursRed = new ArrayList<>();
        List<MatOfPoint> contoursBlue = new ArrayList<>();


        // erode and dilate to remove noise
        Mat kernel = Mat.ones(5, 5, CvType.CV_8U);
        Mat morphedImage = new Mat(rgbaFrame.size(), rgbaFrame.type());

        Imgproc.morphologyEx(rgbaFrame, morphedImage, Imgproc.MORPH_OPEN, kernel);

        Imgproc.cvtColor(morphedImage, redHSV, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(morphedImage, blueHSV, Imgproc.COLOR_RGB2HSV);

        // thresholding for red ball
        ImageUtil.hsvInRange(redHSV, redHsvLower, redHsvUpper, thresholdedHSVRed);

        // thresholding for blue ball
        ImageUtil.hsvInRange(blueHSV, blueHsvLower, blueHsvUpper, thresholdedHSVBlue);

        //Log.d(TAG, "HSV: " + hsv.toString());

        Imgproc.findContours(thresholdedHSVRed, contoursRed, hierachy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(thresholdedHSVBlue, contoursBlue, hierachy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//        Mat matOnlyContours = new Mat(thresholdedHSV.size(), thresholdedHSV.type());
//
//        Imgproc.drawContours(matOnlyContours, contours, -1, new Scalar(100, 100, 255));
        if (contoursRed.size() > 0 && contoursBlue.size() > 0) {
            Log.i(TAG, "Red Contour: " + contoursRed.size());
            MatOfPoint largestContourRed = findLargestContour(contoursRed);
            MatOfPoint largestContourBlue = findLargestContour(contoursBlue);

            double areaBlue = Imgproc.contourArea(largestContourBlue);
            double areaRed = Imgproc.contourArea(largestContourRed);
            Moments mRed = Imgproc.moments(largestContourRed);
            Moments mBlue = Imgproc.moments(largestContourBlue);

            int centerXBlue = (int) (mBlue.get_m10() / mBlue.get_m00());
            int centerYBlue = (int) (mBlue.get_m01() / mBlue.get_m00());

            int centerXRed = (int) (mRed.get_m10() / mRed.get_m00());
            int centerYRed = (int) (mRed.get_m01() / mRed.get_m00());

            Imgproc.circle(rgbaFrame, new Point(centerXBlue, centerYBlue), 3, new Scalar(100, 100, 100));
            Imgproc.circle(rgbaFrame, new Point(centerXRed, centerYRed), 3, new Scalar(100, 100, 100));

            Jewel redJewel = new Jewel(areaRed, BallColor.RED, centerXRed, centerYRed);
            Jewel blueJewel = new Jewel(areaBlue, BallColor.BLUE, centerXBlue, centerYBlue);

            // i could combine these but too lazy
            return new ImageProcessorResult<>(startTime, thresholdedHSVBlue, new BallCenterResult(redJewel, blueJewel));
        } else {
            Log.i(TAG, "No contours found");
            blueHSV.release();
            redHSV.release();
            hierachy.release();

            return new ImageProcessorResult<>(startTime, thresholdedHSVBlue, new BallCenterResult());
        }
    }

    public void changeSLower(int value) {
        blueHsvLower = new Scalar(99, value, 0);
    }

//    public void changeRedLower(int h, int s, int v) {
//        blueHsvLower = new Scalar(h, s ,v);
//    }

    public void changeSUpper(int value) {
        blueHsvUpper = new Scalar(113, value, 255);
    }



    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        MatOfPoint largest = contours.get(0);

        for (MatOfPoint contour : contours) {
            if (contour.size().area() > largest.size().area()) {
                largest = contour;
            }
        }

        return largest;
    }

    // test circle detection stuff for removing error
//    private List<MatOfPoint2f> findSphericalContour(List<MatOfPoint> contours) {
//        MatOfPoint2f approx = new MatOfPoint2f();
//        List<MatOfPoint2f> spheres = new ArrayList<>();
//
//        for (MatOfPoint point : contours) {
//            MatOfPoint2f point2f = new MatOfPoint2f(point.toArray());
//            Imgproc.HoughCircles();
//
//        }
//
//    }
}
