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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

@TeleOp(name="opencv", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class OpenCVTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private FrameGrabber frame = FtcRobotControllerActivity.frameGrabber;

    private int h = 0;
    private int s = 226;
    private int v = 85;
    private boolean add = true;
    private int addAmt = 1;
    private int hUpper = 179;

    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        BallCenterProcessor processor = new BallCenterProcessor(true);
        frame.setImageProcessor(processor);
        waitForStart();
        runtime.reset();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

        // run until the end of the match (driver presses STOP)
        int lowerCount = 0;
        int upperCount = 0;
        while (opModeIsActive()) {

            if (gamepad1.a) {
                lowerCount++;
            } else if (gamepad1.y) {
                upperCount = 0;
            }

//            processor.changeHLower(lowerCount);
//            processor.changeHUpper(upperCount);

            if (gamepad1.x) {
                if (add)
                    h += addAmt;
                else
                    h -= addAmt;
            }

            if (gamepad1.y) {
                if (add)
                    s += addAmt;
                else
                    s -= addAmt;
            }

            if (gamepad1.b) {
//                if (add)
//                    v += addAmt;
//                else
//                    v -= addAmt;
                if (add)
                    v += addAmt;
                else
                    v -= addAmt;
            }

            if (gamepad1.a) {
                add = !add;
            }

            if (gamepad1.left_bumper) {
                addAmt = 1;
            }

            if (gamepad1.right_bumper) {
                addAmt = 50;
            }

            if (gamepad1.dpad_up) {
                h+= addAmt;
            }

            if (gamepad1.dpad_down) {
                h-= addAmt;
            }

            if (gamepad1.dpad_left) {
                hUpper -= addAmt;
            }

            if (gamepad1.dpad_right) {
                hUpper += addAmt;
            }

//            //processor.changeRedLower(h, s, v);
//            processor.changeSLower(h);
//            processor.changeSUpper(hUpper);



            frame.grabSingleFrame();

            while (!frame.isResultReady()) {
                try {
                    Thread.sleep(5); //sleep for 5 milliseconds wait for thing to be ready
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            BallCenterResult result = (BallCenterResult) frame.getResult().getResult();

//            telemetry.addData("S Lower", h);
//            telemetry.addData("S Upper", hUpper);
            if (result.isFoundResult()) { // check if found anything
//                telemetry.addData("Status", "X " + result.getxCoord());
//                telemetry.addData("Status", "Y " + result.getyCoord());
//                telemetry.addData("Area", result.getArea());
                telemetry.addData("Blue X: ", result.getBlue().getCenterX());
                telemetry.addData("Right Jewel: ", result.getRightJewel());
                telemetry.addData("Left Jewel: ", result.getLeftJewel());
            } else {
                telemetry.addData("Status", "No contours found");
            }
//            telemetry.addData("Right: ", result.getRightJewel().getCenterX());
//            telemetry.addData("Left: ", result.getLeftJewel().getCenterX());
          //  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Z: Heading Y: Roll X: Pitch

            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
        }
    }
}
