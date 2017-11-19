/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import ftc.vision.BallCenterProcessor;
import ftc.vision.BallCenterResult;
import ftc.vision.FrameGrabber;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoBlue", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class AutoBlue extends LinearOpMode {
    public final String TAG = "AutoBlue";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private FrameGrabber frame = FtcRobotControllerActivity.frameGrabber;

    // Motors
    private DcMotor leftBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor rightFrontMotor;

    // AndyMark 20: 560
    // AndyMark 40: 1120
    // AndyMark 60: 1680
    // AndyMark 3.7: 103

    // motor encoder calibration
    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI); // circumference

    // telemetry
    private Telemetry.Item debug;

    // vuforia stuff
    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable leftTarget, rightTarget, centerTarget;
    private Pictographs currentTarget;
    private List<VuforiaTrackable> allTrackables;

    private double proportionalGain = .008;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // initialize motors
        leftBackMotor = hardwareMap.dcMotor.get("leftback");
        //leftFrontMotor = hardwareMap.dcMotor.get("leftfront");
        rightBackMotor = hardwareMap.dcMotor.get("rightback");
        //rightFrontMotor = hardwareMap.dcMotor.get("rightfront");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // pre set telemetry stuff
        telemetry.setAutoClear(true);
        debug = telemetry.addData("Debug", 0);
        Telemetry.Item state = telemetry.addData("State", "Init");

        ////////////////////////////////////////////////////
        //                                                //
        //                                                //
        //                 Vuforia Stuff                  //
        //                                                //
        //                                                //
        ////////////////////////////////////////////////////
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ARkaptL/////AAAAGZh9qW5VjUFXqr6Ifl7pO9wYYX/eCbWeNSabmx/9Pyp8LKUH2PgfYHhy5ctv9/d2lZRy4L3KjY6lVgWxezb0lJfZFzmGu3Seuxtdo9/PnBvu0AM6yRptIOR3m79S9K78FGG9aroK9d3KS+WcRBIg7WJYSboPDxnjPlwLT9qSaUFLvi4p9LC1X24ITUHE6nUve2aHM4zQ8i2KQ7hUiFQ9R8dUuk8lfjw4E/bcVWfr8vVMNZx8o/jsQPl5QxH2lth52jQw1tbFVixp4zNJ0PvicmDQftXHIWnGag9NBIi5jOJmUBcfFr22EwCnxQJQ7ZkS4ydZe3uhGtrzYSL5ymsN6FnORvcE2GTrF0XaZpa0Saon\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new VuforiaLocalizerCustomImpl(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        VuforiaTrackables columnLists = this.vuforia.loadTrackablesFromAsset("Cropped_targets2");
        leftTarget = columnLists.get(2);
        leftTarget.setName("leftTarget");  // Left

        centerTarget  = columnLists.get(1);
        centerTarget.setName("centerTarget");  // Center

        rightTarget  = columnLists.get(0);
        rightTarget.setName("rightTarget");  // Right


        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(columnLists);


        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        //float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels


        OpenGLMatrix leftTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation( 0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 0, 0));
        leftTarget.setLocation(leftTargetLocationOnField);
        RobotLog.ii(TAG, "Left Target=%s", format(leftTargetLocationOnField));


        OpenGLMatrix centerTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 0, 0));
        centerTarget.setLocation(centerTargetLocationOnField);
        RobotLog.ii(TAG, "Center Target=%s", format(centerTargetLocationOnField));


        OpenGLMatrix rightTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 0, 0));
        rightTarget.setLocation(rightTargetLocationOnField);
        RobotLog.ii(TAG, "Right Target=%s", format(rightTargetLocationOnField));


        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));


        ((VuforiaTrackableDefaultListener)leftTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)centerTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)rightTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        columnLists.activate();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            state.setValue("Jewel Mode");
//            moveToBall();
            Pictographs pic = detectPictograph();
            currentTarget = pic;
            telemetry.addData("Pic", pic);
            telemetry.update();

            realignWithPicto();
        }
    }

    private Pictographs detectPictograph() {
        if (((VuforiaTrackableDefaultListener)leftTarget.getListener()).isVisible()) {
            return Pictographs.LEFT;
        } else if (((VuforiaTrackableDefaultListener)centerTarget.getListener()).isVisible()) {
            return Pictographs.CENTER;
        } else if (((VuforiaTrackableDefaultListener)rightTarget.getListener()).isVisible()) {
            return Pictographs.RIGHT;
        } else {
            return Pictographs.NOTHING;
        }

        //TODO: last location thing
        // haha
    }

    private void moveToBall() throws InterruptedException {
        BallCenterProcessor processor = new BallCenterProcessor(true);

        Telemetry.Item ballStatus = telemetry.addData("Status", "Beginning move ball");
        setMotorNormal();
        double area = 0; // arbitrary definition, area of the ball
        //frame.setImageProcessor(new BallCenterProcessor(true)); // set true for red, false for blue

        // threshold values
        final int CENTER_POSITION_THRESH = 20; // give some threshold for center
        final int CENTER_POSITION = 110; // "center" value for the ball
        System.out.println("Entering ball loop...");
        vuforia.setFrameQueueCapacity(1);

        do {
            System.out.println("Entered Loop");
            VuforiaLocalizer.CloseableFrame vuforiaFrame = vuforia.getFrameQueue().take();
            Mat openCVFrame = vuforiaToOpenCV(vuforiaFrame);

            System.out.println("gonna process");

//            while (!frame.isResultReady()) {
//                try {
//                    Thread.sleep(5); //sleep for 5 milliseconds wait for thing to be ready
//                    telemetry.addData("Ball", "Waiting for result: " + count);
//                    telemetry.update();
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                sleep(1000);
//            }

            BallCenterResult result = processor.process(System.currentTimeMillis(), openCVFrame, false).getResult();
            telemetry.addData("Status", result.toString());
            telemetry.update();

            if (result.isFoundResult()) { // make sure something is found
                area = result.getArea();
                int x = result.getxCoord();
                int offset = CENTER_POSITION - x; // "error" amount; negative = too left, positive = too right

                if (offset < 0 && Math.abs(offset) > CENTER_POSITION_THRESH) { // too much to the left
                    //strafeRightFor(1, 0.7);
                    ballStatus.setValue("Too much left");
                } else if (offset > 0 && Math.abs(offset) > CENTER_POSITION_THRESH) { // too much to the right
                    //strafeLeftFor(1, 0.7);
                    ballStatus.setValue("Too much right");
                } else {
                    //forwardFor(3, 0.7); // GO GO GO
                    ballStatus.setValue("Good");
                }

                debug.setValue(offset);
                telemetry.update();

            } else {
                ballStatus.setValue("Can't find balls, this is bad");
                telemetry.update();
            }

        } while (opModeIsActive()); // when we are close enough get outta here area < 6000
    }

    private void setMotorNormal() {
        //leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setMotorRunToPos() {
        //leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetMotors() {
       // leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void normalDrive(double leftFront, double leftBack, double rightFront, double rightBack, long time) {
        setMotorNormal();
      //  leftFrontMotor.setPower(leftFront);
        leftBackMotor.setPower(leftBack);
       // rightFrontMotor.setPower(rightFront);
        rightBackMotor.setPower(rightBack);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        stopMotors();
    }

    private void stopMotors() {
        normalDrive(0, 0, 0, 0);
    }


    private void strafeLeftFor(int inches, double speed) {
        encoderDrive(-inches, inches, -inches, inches, speed);
    }

    private void strafeRightFor(int inches, double speed) {
        encoderDrive(inches, -inches, inches, -inches, speed);
    }

    private void forwardFor(int inches, double speed) {
        encoderDrive(inches, inches, inches, inches, speed);
    }

    private void rotateRight(int time, double speed) {
        setMotorNormal();

        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        stopMotors();
    }

    private void rotateLeft(int time, double speed) {
        setMotorNormal();
        leftBackMotor.setPower(-speed);
        rightBackMotor.setPower(-speed);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        stopMotors();
    }

    private void realignWithPicto() {
        for (VuforiaTrackable trackable : allTrackables) {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            //telemetry.addData("realignWithPicto", "Success");

            // found the pictograph

            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
              //  System.out.println(lastLocation.getData());
               // telemetry.addData("lastLocation", "success");
            } else if (lastLocation == null) {
               // telemetry.addData("lastLocation", "fail");
                continue;
            }

            double yaw, x, y;
            y = Double.parseDouble(finalize(format(lastLocation))[5]);


            RealignState realignState = RealignState.YAW;

            telemetry.update();
            boolean isDone = false;
            // do this until we are good
            do {

                robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                //TODO find whats wrong with it repeating
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

                String[] data = finalize(format(lastLocation));
                yaw = Double.parseDouble(data[1]);
                x = Double.parseDouble(data[3]);
                y = Double.parseDouble(data[5]);

                if (realignState == RealignState.YAW) {                 // fix the yaw by rotating itself till it's good
                    telemetry.addData("RealignStatus", "Fixing Yaw");
                    telemetry.addData("Yaw", yaw);
                    if (fixYaw(yaw)) {
                        realignState = RealignState.X;
                    }
                } else if (realignState == RealignState.X) {            // fix the X by strafing till it's good
                    telemetry.addData("RealignStatus", "Fixing X");
                    telemetry.addData("X", x);
                    realignState = RealignState.Y;
//                    if (fixX(x)) {
//                        realignState = RealignState.Y;
//                    }
                } else if (realignState == RealignState.Y) {            // fix the Y by simply driving forward
                    telemetry.addData("RealignStatus", "Fixing Y");
                    telemetry.addData("Y", y);
                    if (fixY(y)) {
                        isDone = true;
                    }
                }

                telemetry.update();
            } while (!isDone && opModeIsActive());

        }
        stopMotors();
    }

    private boolean fixYaw(double yaw) {
        if (yaw < -5) {
            rotateLeft((int) 100 , (Math.abs(yaw) * proportionalGain));
            telemetry.addData("Error:" , -(yaw));
            telemetry.addData("Motor", -yaw*proportionalGain);
            telemetry.update();
        } else if (yaw<-2.5 && yaw>-5){
            rotateRight((int) 70 , (Math.abs(yaw) * proportionalGain *.8));
            telemetry.addData("Error:" , yaw);
            telemetry.addData("Motor", -yaw*proportionalGain);
            telemetry.update();
        } else if (yaw > 5){
            rotateRight((int) 100 , (Math.abs(yaw) * proportionalGain));
            telemetry.addData("Error:" , yaw);
            telemetry.addData("Motor", -yaw*proportionalGain);
            telemetry.update();
        } else if (yaw>2.5 && yaw<5){
            rotateRight((int) 70 , (Math.abs(yaw) * proportionalGain *.8));
            telemetry.addData("Error:" , yaw);
            telemetry.addData("Motor", -yaw*proportionalGain);
            telemetry.update();
        } else {
            return true;
        }
        return false;
    }

    private boolean fixX(double x) {
        if (x > 0) {
            strafeLeftFor(1, 0.5);
        } else if (x < 0) {
            strafeRightFor(1, 0.5);
        } else {
            return true;
        }

        return false;
    }

    private boolean fixY(double y) {
        if (y > 0.27) {
            //normalDrive(0, 0.08, 0, -0.08, 10);
        } else {
            return true;
        }
        return false;
    }




    private void encoderDrive(int leftFront, int leftBack, int rightFront, int rightBack, double power) {
        setMotorRunToPos();
        resetMotors();

      //  leftFrontMotor.setTargetPosition((int)(leftFront * COUNTS_PER_INCH));
        leftBackMotor.setTargetPosition((int)(leftBack * COUNTS_PER_INCH));
       // rightFrontMotor.setTargetPosition((int)(rightFront * COUNTS_PER_INCH));
        rightBackMotor.setTargetPosition((int)(rightBack * COUNTS_PER_INCH));

       // leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
      //  rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);

        // hang until they're done
        while (leftBackMotor.isBusy() && rightBackMotor.isBusy()) { // TODO add in front motors
            //telemetry.addData("Left Front: ", leftFrontMotor.getCurrentPosition());
            telemetry.addData("Left Back: ", leftBackMotor.getCurrentPosition());
            //telemetry.addData("Right Front: ", rightFrontMotor.getCurrentPosition());
            telemetry.addData("Right Back: ", rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
    }

    private Mat vuforiaToOpenCV(VuforiaLocalizer.CloseableFrame vuforiaFrame) {
        System.out.println("Converting vuforia frame to OpenCV frame");
        long numImages = vuforiaFrame.getNumImages();
        Image rgb = null;

        System.out.println(vuforiaFrame.getNumImages());

        // apparently all of the frames are greyscale?
        // set it to BGR using static method in Vuforia class
        for (int i = 0; i < numImages; i++) {
            System.out.println("Format: " + vuforiaFrame.getImage(i).getFormat());
            if (vuforiaFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = vuforiaFrame.getImage(i);
                break;
            }
        }

        /*rgb is now the Image object that weve used in the video*/
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        //put the image into a MAT for OpenCV
        Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, tmp);

        //close the frame, prevents memory leaks and crashing
        vuforiaFrame.close();

        return tmp;
    }

    private String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    private String [] finalize(String strPos){

        strPos = strPos.replaceAll("[^0-9\\s+.-]", "");
        strPos = strPos.trim();
        strPos = strPos +" ";
        String[] values = strPos.split("\\s+");
        return values;


    }

    enum Pictographs {
        CENTER,
        LEFT,
        RIGHT,
        NOTHING
    }

    enum RealignState {
        YAW,
        X,
        Y,
        NOTHING
    }


}
