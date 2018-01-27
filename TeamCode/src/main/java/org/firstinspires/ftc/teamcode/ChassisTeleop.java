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
    private Servo left;
    private Servo right;
    private Servo spin;

    private double leftPos = 0;
    private double rightPos = 0;

    private DcMotor lifter;
    private int lifterPos;
    private final int LIFTER_MAX = -10500; //-10916
    private DigitalChannel lifterButton;

    private BNO055IMU imu;
    private boolean firstStrafe  = true;
    private float firstStrafeHeading;

    private final double flickUpPosition = 0.2;
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

        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
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
        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);

        //smash = MediaPlayer.create(hardwareMap.appContext, R.raw.smash);
        //running = MediaPlayer.create(hardwareMap.appContext, R.raw.running);

        // Wait for the game to start (driver presses PLAY)

//        leftClamp.setPosition(1);
//        rightClamp.setPosition(0);


        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MediaPlayer error = MediaPlayer.create(hardwareMap.appContext, R.raw.errormessage);
        MediaPlayer chime = MediaPlayer.create(hardwareMap.appContext, R.raw.chimeconnect);
        MediaPlayer player = MediaPlayer.create(hardwareMap.appContext, R.raw.warningmessage);

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

        boolean attop = true;

        player.start();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // update the current angle
            Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Z: Heading Y: Roll X: Pitch

            left.setPosition(0.37);
            right.setPosition(0.6);
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

//haha

            if (gamepad1.dpad_right) {
                if (firstStrafe)
                    firstStrafeHeading = angles.firstAngle; // record the original heading

                firstStrafe = false;
                strafeRightFor(0.8, angles.firstAngle);
            } else if (gamepad1.dpad_left) {
                if (firstStrafe)
                    firstStrafeHeading = angles.firstAngle;

                firstStrafe = false;
                strafeLeftFor(0.8, angles.firstAngle);

            } else {
                firstStrafe = true;
                rightfrontMotor.setPower(-gamepad1.right_stick_y * slowFactor);
                rightbackMotor.setPower(-gamepad1.right_stick_y * slowFactor);

                leftfrontMotor.setPower(-gamepad1.left_stick_y * slowFactor);
                leftbackMotor.setPower(-gamepad1.left_stick_y * slowFactor);
            }

            // open: left = 1
            // open: right = 0



            if (gamepad2.dpad_up) {
                lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //if (lifterPos > 100)r
                lifter.setPower(1);
            } else if (gamepad2.dpad_down) {
                //if (lifterPos < LIFTER_MAX)
                lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                lifter.setPower(-0.5);

                telemetry.addData("Position: ", lifter.getCurrentPosition());
                telemetry.update();
            } else {
                lifter.setPower(0);
            }




            //telemetry.addData("Servo Max: ", leftClamp.MAX_POSITION);
            //telemetry.addData("Servo Min: ", leftClamp.MIN_POSITION);

            //telemetry.addData("Right Position: ", rightClamp.getPower());



            lifterPos = lifter.getCurrentPosition();

            if (gamepad2.b) {
                resetLifterMotor();
                lifter.setTargetPosition(5000);
                lifter.setPower(1);
            }

            if (gamepad2.x) {
                resetLifterMotor();
                lifter.setTargetPosition(10000);
                lifter.setPower(1);
            }

            if(gamepad2.y){
                resetLifterMotor();

                lifter.setTargetPosition(20000);
                lifter.setPower(1);

                if (attop) {
                    spin.setPosition(1);
                    attop = false;
                } else {
                    spin.setPosition(-1);
                    attop = true;
                }
            }

            if(gamepad2.right_bumper) {
                botr.setPower(1);
                botl.setPower(-1);
                topr.setPower(-1);
                topl.setPower(1);
            } else if (gamepad2.left_bumper) {
                botr.setPower(-1);
                botl.setPower(1);
                topr.setPower(1);
                topl.setPower(-1);
            }
            else {
                botr.setPower(0);
                botl.setPower(0);
                topr.setPower(0);
                topl.setPower(0);
            }


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