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
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private CRServo topr;
    private CRServo topl;
    private CRServo botr;
    private CRServo botl;
    private Servo leftBottom;
    private Servo rightBottom;
    private Servo spin;
    private Servo leftTop;
    private Servo rightTop;

    private double leftPos = 0;
    private double rightPos = 0;

    private DcMotor lifter;
    private int lifterPos;
    private final int LIFTER_MAX = -10500; //-10916
    private DigitalChannel lifterButton;

    private BNO055IMU imu;
    private boolean firstStrafe  = true;
    private boolean spinDebouncer= false;

    private float firstStrafeHeading;

    private final double flickUpPosition = 0.2;
    private final float strafeKP = 0.05f;
    private double slowFactor = 1;

    private boolean relicMode = false;

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

        leftBottom = hardwareMap.servo.get("leftbottom");
        rightBottom = hardwareMap.servo.get("rightbottom");
        rightTop = hardwareMap.servo.get("righttop");
        leftTop= hardwareMap.servo.get("lefttop");
        topr = hardwareMap.crservo.get("topr");
        topl = hardwareMap.crservo.get("topl");
        botr = hardwareMap.crservo.get("botr");
        botl = hardwareMap.crservo.get("botl");
        spin = hardwareMap.servo.get("spin");

        lifter = hardwareMap.dcMotor.get("lifter");
        // lifterButton = hardwareMap.digitalChannel.get("button");

        leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftfrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightfrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightbackMotor.setDirection(DcMotor.Direction.REVERSE);

        //smash = MediaPlayer.create(hardwareMap.appContext, R.raw.smash);
        //running = MediaPlayer.create(hardwareMap.appContext, R.raw.running);

        // Wait for the game to start (driver presses PLAY)

//        leftClamp.setPosition(1);
//        rightClamp.setPosition(0);


        //lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MediaPlayer error = MediaPlayer.create(hardwareMap.appContext, R.raw.errormessage);
        MediaPlayer chime = MediaPlayer.create(hardwareMap.appContext, R.raw.chimeconnect);
        MediaPlayer player = MediaPlayer.create(hardwareMap.appContext, R.raw.warningmessage);
        MediaPlayer smash = MediaPlayer.create(hardwareMap.appContext, R.raw.allstar);

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
        Servo flicker;
        flicker = hardwareMap.servo.get("flick");
        flicker.setPosition(flickUpPosition);
        float left_y = 0;
        float right_y = 0;

        boolean attop = true;
        if (spin.getPosition() > 0.5)
            attop = false;
        if (attop)
            spin.setPosition(0);
        else
            spin.setPosition(1);


        // collect
        botr.setPower(1);
        botl.setPower(-1);
        topr.setPower(-1);
        topl.setPower(1);
        sleep(300);

        smash.start();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // update the current angle
            Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Z: Heading Y: Roll X: Pitch


            // controls for controlling top and bottom arms2
            if (gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.left_stick_x < -.1 || gamepad2.left_stick_x > .1 || gamepad2.right_stick_x < -.1 || gamepad2.right_stick_x > .1) {
                if (gamepad2.dpad_right) { // bring top out
                    leftTop.setPosition(0.0);
                    rightTop.setPosition(1.0);

                } else if (gamepad2.dpad_left) {  // bring top in
                    leftTop.setPosition(0.55);
                    rightTop.setPosition(0.35);
                }

                if (gamepad2.left_stick_x < -.1 || gamepad2.right_stick_x < -.1) { // bring bottom in
                    leftBottom.setPosition(0.35);
                    rightBottom.setPosition(0.55);

                    telemetry.addData("Left Stick: ", gamepad2.left_stick_x);
                } else if (gamepad2.right_stick_x > 0.1 || gamepad2.left_stick_x > 0.1) { // bring bottom out
                    leftBottom.setPosition(1.0);
                    rightBottom.setPosition(0.0);
                }


            } else {    // controls for both of the arms

                if (gamepad2.b) { // Open grippers
                    leftTop.setPosition(0.0);
                    leftBottom.setPosition(1.0);

                    rightTop.setPosition(1.0); // fix the top
                    rightBottom.setPosition(0.0);
                } else if (gamepad1.right_bumper) {
                    leftBottom.setPosition(0.35);
                    leftTop.setPosition(0.55);

                    rightBottom.setPosition(0.55);
                    rightTop.setPosition(0.35);
                } else {
                    leftBottom.setPosition(0.55);
                    leftTop.setPosition(0.31);

                    rightTop.setPosition(0.59);
                    rightBottom.setPosition(0.35);
                }
            }


            if (gamepad1.left_bumper) { // Slow mode driving
                slowFactor = 0.35;
            } else {
                slowFactor = 1;
            }


            //if (gamepad2.a) {/
            //    chime.start();
            //} else if (gamepad2.b) {
            //    error.start();
            //}

            // movement controls
            if (gamepad1.dpad_right) {
//                if (firstStrafe)
//                    firstStrafeHeading = angles.firstAngle; // record the original heading
//
//                firstStrafe = false;
                strafeRightFor(0.5, angles.firstAngle);
            } else if (gamepad1.dpad_left) {
//                if (firstStrafe)
//                    firstStrafeHeading = angles.firstAngle;
//
//                firstStrafe = false;
                strafeLeftFor(0.5, angles.firstAngle);

            } else {
                // Allow gamepad 1 to supersede gamepad 2
                if (Math.abs(gamepad1.right_stick_y)>.1 || Math.abs(gamepad1.left_stick_y) > .1 ){
                    left_y = gamepad1.left_stick_y;
                    right_y = gamepad1.right_stick_y;
                }
//                else if(Math.abs(gamepad2.right_stick_y) > .1 || Math.abs(gamepad2.left_stick_y) > .1){ // get rid of this for now
//                    left_y = gamepad2.left_stick_y;
//                    right_y = gamepad2.right_stick_y;
//                }
                else {
                    left_y = 0;
                    right_y = 0;
                }
                rightfrontMotor.setPower(-right_y * slowFactor);
                rightbackMotor.setPower(-right_y * slowFactor);
                leftfrontMotor.setPower(-left_y * slowFactor);
                leftbackMotor.setPower(-left_y * slowFactor);
            }


            // open: left = 1
            // open: right = 0



            if (gamepad2.dpad_down) {
                lifter.setPower(0.8);
            } else if (gamepad2.dpad_up) {
                lifter.setPower(-1);
            } else {
                lifter.setPower(0);
            }

            // spin it
            if(gamepad2.y) {
                if(!spinDebouncer) {
                    if (attop) {
                        spin.setPosition(1);
                        attop = false;
                    } else {
                        spin.setPosition(0);
                        attop = true;
                    }
                    spinDebouncer = true;
                }
            }
            else
                spinDebouncer = false;

            if (gamepad1.right_trigger > 0) {
                    botr.setPower(-1);
                    botl.setPower(1);
                    topr.setPower(1);
                    topl.setPower(-1);
            }


            // Gripper control
            if(gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0 || gamepad1.right_trigger > 0) {
                if (gamepad2.a) { // Rotate block.
                    if (attop && gamepad2.right_bumper || !attop && gamepad2.left_trigger > 0) {
                        topr.setPower(-1);
                        topl.setPower(-1);
                    } else if (attop && gamepad2.left_bumper || !attop && gamepad2.right_trigger > 0) {
                        topr.setPower(1);
                        topl.setPower(1);
                    }
                    if (attop && gamepad2.right_trigger > 0 || !attop && gamepad2.left_bumper) {
                        botr.setPower(1);
                        botl.setPower(1);
                    } else if (attop && gamepad2.left_trigger > 0 || !attop && gamepad2.right_bumper) {
                        botr.setPower(-1);
                        botl.setPower(-1);
                    }
                } else { // Normal intake / outtake
                    if (!attop && gamepad2.right_bumper || attop && gamepad2.right_trigger > 0) {
                        botr.setPower(1);
                        botl.setPower(-1);
                    } else if (!attop && gamepad2.left_bumper || attop && gamepad2.left_trigger > 0) {
                        botr.setPower(-1);
                        botl.setPower(1);
                    }
                    if (!attop && gamepad2.right_trigger > 0 || attop && gamepad2.right_bumper) {
                        topr.setPower(-1);
                        topl.setPower(1);
                    } else if (!attop && gamepad2.left_trigger > 0 || attop && gamepad2.left_bumper) {
                        topr.setPower(1);
                        topl.setPower(-1);
                    }
                }
            }
            else {
                topr.setPower(0);
                topl.setPower(0);
                botr.setPower(0);
                botl.setPower(0);
            }

            telemetry.addData("attop: ", attop);
            telemetry.update();

        }
    }

    private void strafeRightFor(double power, float heading) {
        float error = firstStrafeHeading - heading;
        float factor = error * strafeKP;

        if (factor < 0)
            factor = 0.5f;
        else
            factor = 1.5f;

        leftbackMotor.setPower(power * slowFactor);
        leftfrontMotor.setPower((-power * slowFactor));// + (error * strafeKP)); // cgabge tgus
        rightbackMotor.setPower(-power * slowFactor);
        rightfrontMotor.setPower((power * slowFactor));// + (error * strafeKP)); // affected

        telemetry.addData("error: ", error);
        telemetry.addData("factor: ", factor);
        telemetry.addData("heading: ", heading);
        telemetry.addData("first heading: ", firstStrafeHeading);
        telemetry.addData("firststrafe", firstStrafe);
        telemetry.update();
    }

    private void strafeLeftFor(double power, float heading) {
        float error = firstStrafeHeading - heading;
        float factor = (error * strafeKP) + 1;

        leftbackMotor.setPower(-power * slowFactor);
        leftfrontMotor.setPower((power * slowFactor));// + (error * strafeKP));
        rightbackMotor.setPower(power * slowFactor);
        rightfrontMotor.setPower((-power * slowFactor));// + (error * strafeKP));

        telemetry.addData("error: ", error);
        telemetry.addData("factor: ", factor);
        telemetry.addData("heading: ", heading);
        telemetry.addData("first heading: ", firstStrafeHeading);
        telemetry.update();
    }

    private void resetLifterMotor() {
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}