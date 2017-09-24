package ftc.vision;

import android.util.Log;

import org.opencv.core.Core;
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
    private final Scalar redHsvLower = new Scalar(0, 130, 106);
    private final Scalar redHsvUpper = new Scalar(15, 255, 255);

    private final Scalar blueHsvLower = new Scalar(98, 117, 59);
    private final Scalar blueHsvUpper = new Scalar(114, 225, 255);

//    private final Scalar blueHsvLower = new Scalar(0, 0, 0);
//    private final Scalar blueHsvUpper = new Scalar(179, 255, 255);

    public BallCenterProcessor(boolean detectRed) {
        this.detectRed = detectRed;
    }

    @Override
    public ImageProcessorResult<BallCenterResult> process(long startTime, Mat rgbaFrame, boolean saveImages) {
        Mat hsv = rgbaFrame.clone();
        Mat hierachy = rgbaFrame.clone();
        Mat thresholdedHSV = rgbaFrame.clone();

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.cvtColor(rgbaFrame, hsv, Imgproc.COLOR_RGB2HSV);

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
            Moments m = Imgproc.moments(largestContour);

            int center_x = (int) (m.get_m10() / m.get_m00());
            int center_y = (int) (m.get_m01() / m.get_m00());

            Imgproc.circle(rgbaFrame, new Point(center_x, center_y), 3, new Scalar(100, 100, 100));

            return new ImageProcessorResult<>(startTime, thresholdedHSV, new BallCenterResult(center_x, center_y));
        } else {
            Log.i(TAG, "No contours found");
            hsv.release();
            hierachy.release();

            return new ImageProcessorResult<>(startTime, thresholdedHSV, new BallCenterResult());
        }
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
