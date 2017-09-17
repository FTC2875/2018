package ftc.vision;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ftc-2875 on 9/17/17.
 */

public class BallCenterProcessor implements ImageProcessor<BallCenterResult> {

    private final String TAG = "Image Processor";
    private boolean detectRed;
    private final Scalar redHsvLower = null;
    private final Scalar redHsvUpper = null;

    private final Scalar blueHsvLower = new Scalar(103, 100, 112);
    private final Scalar blueHsvUpper = new Scalar(118, 220, 255);

    public BallCenterProcessor(boolean detectRed) {
        this.detectRed = detectRed;
    }

    @Override
    public ImageProcessorResult<BallCenterResult> process(long startTime, Mat rgbaFrame, boolean saveImages) {
        Log.i(TAG, "Starting to process");
        Mat hsv = rgbaFrame.clone();
        Mat hierachy = rgbaFrame.clone();

        List<MatOfPoint> contours = new ArrayList<>();

        if (!detectRed) {
            // thresholding for blue ball
            Core.inRange(rgbaFrame, blueHsvLower, blueHsvUpper, hsv);
        } else {
            // thresholding for red ball
        }

        Imgproc.findContours(hsv, contours, hierachy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 0) {
            MatOfPoint largestContour = findLargestContour(contours);
            Moments m = Imgproc.moments(largestContour);

            int center_x = (int) (m.get_m10() / m.get_m00());
            int center_y = (int) (m.get_m01() / m.get_m00());

            Imgproc.circle(hsv, new Point(center_x, center_y), 3, new Scalar(100, 100, 100));
            ImageUtil.saveImage(TAG, hsv, Imgproc.COLOR_HSV2BGR, "thresh", startTime);

            return new ImageProcessorResult<>(startTime, hsv, new BallCenterResult(center_x, center_y));
        } else {
            Log.i(TAG, "No contours found");
            hsv.release();
            hierachy.release();

            return null;
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
}
