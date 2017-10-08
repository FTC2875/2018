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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

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

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private FrameGrabber frame = FtcRobotControllerActivity.frameGrabber;

    // Motors
    private DcMotor leftBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor rightFrontMotor;

    // threshold values
    private final int CENTER_POSITION_THRESH = 10; // give some threshold
    private final int CENTER_POSITION = 270; // "center" value for the ball

    // motor encoder calibration
    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // initialize motors
        leftBackMotor = hardwareMap.dcMotor.get("leftback");
        leftFrontMotor = hardwareMap.dcMotor.get("leftfront");
        rightBackMotor = hardwareMap.dcMotor.get("rightback");
        rightFrontMotor = hardwareMap.dcMotor.get("rightfront");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }

    private void moveToBall() {
        setMotorNormal();
        double area = 0; // arbitrary definition, area of the ball
        frame.setImageProcessor(new BallCenterProcessor(true)); // set true for red, false for blue

        do {
            frame.grabSingleFrame();
            BallCenterResult result = (BallCenterResult) frame.getResult().getResult();

            if (result.isFoundResult()) { // make sure something is found
                area = result.getArea();
                int x = result.getxCoord();
                int offset = CENTER_POSITION - x;

                if (offset > 0 && offset > CENTER_POSITION_THRESH) { // too much to the left
                    strafeRightFor(1, 0.7);
                } else if (offset < 0 && Math.abs(offset) > CENTER_POSITION_THRESH) { // too much to the right
                    strafeLeftFor(1, 0.7);
                } else {
                    forwardFor(3, 0.7); // GO GO GO
                }

            } else {
                telemetry.addData("Status: ", "Can't find balls, this is bad");
                telemetry.update();
            }

        } while (area < 6000); // when we are close enough get outta here
    }

    private void setMotorNormal() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setMotorRunToPos() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetMotors() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void normalDrive(double leftFront, double leftBack, double rightFront, double rightBack) {
        setMotorNormal();
        leftFrontMotor.setPower(leftFront);
        leftBackMotor.setPower(leftBack);
        rightFrontMotor.setPower(rightFront);
        rightBackMotor.setPower(rightBack);
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

    private void encoderDrive(int leftFront, int leftBack, int rightFront, int rightBack, double power) {
        setMotorRunToPos();
        resetMotors();

        leftFrontMotor.setTargetPosition((int)(leftFront * COUNTS_PER_INCH));
        leftBackMotor.setTargetPosition((int)(leftBack * COUNTS_PER_INCH));
        rightFrontMotor.setTargetPosition((int)(rightFront * COUNTS_PER_INCH));
        rightBackMotor.setTargetPosition((int)(rightBack * COUNTS_PER_INCH));

        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);

        // hang until they're done
        while (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && rightFrontMotor.isBusy()) {
            telemetry.addData("Left Front: ", leftFrontMotor.getCurrentPosition());
            telemetry.addData("Left Back: ", leftBackMotor.getCurrentPosition());
            telemetry.addData("Right Front: ", rightFrontMotor.getCurrentPosition());
            telemetry.addData("Right Back: ", rightBackMotor.getCurrentPosition());
            telemetry.update();
        }


    }



}
