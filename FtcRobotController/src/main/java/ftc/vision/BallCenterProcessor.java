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
    private Scalar redHsvLower = new Scalar(178, 228, 135);
    private Scalar redHsvUpper = new Scalar(179, 255, 238);

    private final Scalar blueHsvLower = new Scalar(98, 117, 59);
    private final Scalar blueHsvUpper = new Scalar(114, 225, 255);

//    private final Scalar blueHsvLower = new Scalar(0, 0, 0);
//    private final Scalar blueHsvUpper = new Scalar(179, 255, 255);

    public BallCenterProcessor(boolean detectRed) {
        this.detectRed = detectRed;
    }

    @Override
    public ImageProcessorResult<BallCenterResult> process(long startTime, Mat rgbaFrame, boolean saveImages) {
        Log.i(TAG, "Starting to image process ball");
        Mat hsv = rgbaFrame.clone();
        Mat hierachy = rgbaFrame.clone();
        Mat thresholdedHSV = rgbaFrame.clone();

        List<MatOfPoint> contours = new ArrayList<>();

        // erode and dilate to remove noise
        Mat kernel = Mat.ones(5, 5, CvType.CV_8U);
        Mat morphedImage = new Mat(rgbaFrame.size(), rgbaFrame.type());

        Imgproc.morphologyEx(rgbaFrame, morphedImage, Imgproc.MORPH_OPEN, kernel);

        Imgproc.cvtColor(morphedImage, hsv, Imgproc.COLOR_RGB2HSV);

        if (!detectRed) {
            // thresholding for blue ball
            //Core.inRange(hsv, blueHsvLower, blueHsvUpper, thresholdedHSV);
            ImageUtil.hsvInRange(hsv, blueHsvLower, blueHsvUpper, thresholdedHSV);
        } else {
            // thresholding for red ball
            ImageUtil.hsvInRange(hsv, redHsvLower, redHsvUpper, thresholdedHSV);
        }

        Imgproc.findContours(thresholdedHSV, contours, hierachy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(thresholdedHSV, contours, -1, new Scalar(100, 100, 255));
        if (contours.size() > 0) {
            Log.i(TAG, "Found at least one contour: " + contours.size());
            MatOfPoint largestContour = findLargestContour(contours);

            double area = Imgproc.contourArea(largestContour);
            Moments m = Imgproc.moments(largestContour);

            int center_x = (int) (m.get_m10() / m.get_m00());
            int center_y = (int) (m.get_m01() / m.get_m00());

            Imgproc.circle(rgbaFrame, new Point(center_x, center_y), 3, new Scalar(100, 100, 100));

            return new ImageProcessorResult<>(startTime, thresholdedHSV, new BallCenterResult(center_x, center_y, area));
        } else {
            Log.i(TAG, "No contours found");
            hsv.release();
            hierachy.release();

            return new ImageProcessorResult<>(startTime, thresholdedHSV, new BallCenterResult());
        }
    }

    public void changeHLower(int value) {
        redHsvLower = new Scalar(value, 130, 106);
    }

    public void changeHUpper(int value) {
        redHsvUpper = new Scalar(15 - value, 255, 255);
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
