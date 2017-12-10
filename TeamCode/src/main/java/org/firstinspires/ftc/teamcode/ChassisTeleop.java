package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@TeleOp(name="HubBotTeleop", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class ChassisTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor rightfrontMotor;
    private DcMotor leftfrontMotor;
    private DcMotor rightbackMotor;
    private DcMotor leftbackMotor;

    private CRServo extender;

    private Servo leftClamp;
    private Servo rightClamp;
    private double leftPos = 0;
    private double rightPos = 0;

    private DcMotor lifter;
    private int lifterPos;
    private final int LIFTER_MAX = -10500; //-10916
    private DigitalChannel lifterButton;

    private BNO055IMU imu;
    private boolean firstStrafe  = true;
    private float firstStrafeHeading;
    private final float strafeKP = 0.05f;

    private double slowFactor = 1;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        rightfrontMotor  = hardwareMap.dcMotor.get("rightfront");
        leftfrontMotor = hardwareMap.dcMotor.get("leftfront");
        rightbackMotor = hardwareMap.dcMotor.get("rightback");
        leftbackMotor = hardwareMap.dcMotor.get("leftback");

        rightClamp = hardwareMap.servo.get("rightclamp");
        leftClamp = hardwareMap.servo.get("leftclamp");

        extender = hardwareMap.crservo.get("extender");

        lifter = hardwareMap.dcMotor.get("lifter");
        // lifterButton = hardwareMap.digitalChannel.get("button");

        leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);

        //smash = MediaPlayer.create(hardwareMap.appContext, R.raw.smash);
        //running = MediaPlayer.create(hardwareMap.appContext, R.raw.running);

        // Wait for the game to start (driver presses PLAY)

        leftClamp.setPosition(1);
        rightClamp.setPosition(0);

        leftPos = leftClamp.getPosition();
        rightPos = rightClamp.getPosition();


        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MediaPlayer error = MediaPlayer.create(hardwareMap.appContext, R.raw.errormessage);
        MediaPlayer chime = MediaPlayer.create(hardwareMap.appContext, R.raw.chimeconnect);
        MediaPlayer player = MediaPlayer.create(hardwareMap.appContext, R.raw.allstar);

        // rev imu stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Z: Heading Y: Roll X: Pitch
            if (gamepad1.left_bumper) {
                slowFactor = 0.35;
            } else {
                slowFactor = 1;
            }

            if (gamepad2.a) {
                chime.start();
            } else if (gamepad2.b) {
                error.start();
            }



            if (gamepad1.dpad_right) {
                if (firstStrafe)
                    firstStrafeHeading = angles.firstAngle; // record the original heading
                else
                    firstStrafe = false;

                    strafeRightFor(1, angles.firstAngle);
            } else if (gamepad1.dpad_left) {
                if (firstStrafe)
                    firstStrafeHeading = angles.firstAngle;
                else
                    firstStrafe = false;

                strafeLeftFor(1, angles.firstAngle);

            } else {
                firstStrafe = true;
                rightfrontMotor.setPower(-gamepad1.right_stick_y * slowFactor);
                rightbackMotor.setPower(-gamepad1.right_stick_y * slowFactor);

                leftfrontMotor.setPower(-gamepad1.left_stick_y * slowFactor);
                leftbackMotor.setPower(-gamepad1.left_stick_y * slowFactor);
            }

            // open: left = 1
            // open: right = 0
            if (gamepad1.x) { // close
                if (rightPos < 1.0)
                    rightPos += 0.05;

                if (leftPos > 0.0)
                    leftPos -= 0.05;
            } else if (gamepad1.b) { // open
                if (rightPos > 0.3)
                    rightPos -= 0.05;

                if (leftPos < .7)
                    leftPos += 0.05;
            }

            if (gamepad1.y) {
                extender.setPower(1);
            } else if (gamepad1.a) {
                extender.setPower(-1);
            } else {
                extender.setPower(0);
            }

            if (gamepad1.dpad_down) {
                //if (lifterPos > 100)
                    lifter.setPower(1);
                error.start();
            } else if (gamepad1.dpad_up) {
                //if (lifterPos < LIFTER_MAX)
                    lifter.setPower(-1);
                error.start();
            } else {
                lifter.setPower(0);
            }



            //telemetry.addData("Servo Max: ", leftClamp.MAX_POSITION);
            //telemetry.addData("Servo Min: ", leftClamp.MIN_POSITION);

            //telemetry.addData("Right Position: ", rightClamp.getPower());


            rightClamp.setPosition(rightPos);
            leftClamp.setPosition(leftPos);
            lifterPos = lifter.getCurrentPosition();

//            telemetry.addData("Left Clamp: ", leftPos);
//            telemetry.addData("Right Clamp: ", rightPos);
//            telemetry.addData("Lifter Pos: ", lifterPos);
//
//            telemetry.addData("Heading:", angles.firstAngle);
//            telemetry.addData("Roll:", angles.secondAngle);
//            telemetry.addData("Pitch:", angles.thirdAngle);
//            telemetry.addData("X Accel", imu.getOverallAcceleration().xAccel);
//            telemetry.addData("Y Accel", imu.getOverallAcceleration().yAccel);
//
//            telemetry.update();
        }
    }

    private void strafeRightFor(double power, float heading) {
        float error = heading - firstStrafeHeading;
        float factor = error * strafeKP;

        leftbackMotor.setPower(power * slowFactor);
        leftfrontMotor.setPower((-power * slowFactor) + factor);// + (error * strafeKP)); // cgabge tgus
        rightbackMotor.setPower(-power * slowFactor);
        rightfrontMotor.setPower((power * slowFactor) + factor);// + (error * strafeKP)); // affected

        telemetry.addData("right error: ", error);
        telemetry.addData("factor: ", factor);
        telemetry.addData("heading: ", heading);
        telemetry.addData("first heading: ", firstStrafeHeading);
        telemetry.update();
    }

    private void strafeLeftFor(double power, float heading) {
        float error = heading - firstStrafeHeading;
        float factor = error * strafeKP;

        leftbackMotor.setPower(-power * slowFactor);
        leftfrontMotor.setPower((power * slowFactor) + factor);// + (error * strafeKP));
        rightbackMotor.setPower(power * slowFactor);
        rightfrontMotor.setPower((-power * slowFactor) + factor);// + (error * strafeKP));

        telemetry.addData("left error: ", error);
        telemetry.addData("factor: ", factor);
        telemetry.addData("heading: ", heading);
        telemetry.addData("first heading: ", firstStrafeHeading);
        telemetry.update();
    }

}