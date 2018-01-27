package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

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
@Disabled
@TeleOp(name="OmniBotDemo", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class OmniDemo extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor rightfrontMotor;
    private DcMotor leftfrontMotor;
    private DcMotor rightbackMotor;
    private DcMotor leftbackMotor;
    private DcMotor raiser;
    private CRServo leftClamp;
    private CRServo rightClamp;

    private DigitalChannel raiserButton;

    private boolean buttonFlag = false;

    private MediaPlayer smash;
    private MediaPlayer running;

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
        raiser = hardwareMap.dcMotor.get("raiser");
        leftClamp = hardwareMap.crservo.get("leftClamp");
        rightClamp = hardwareMap.crservo.get("rightClamp");
        raiserButton = hardwareMap.digitalChannel.get("raiseButton");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);
        raiser.setDirection(DcMotor.Direction.FORWARD);
        leftClamp.setDirection(CRServo.Direction.FORWARD);
        rightClamp.setDirection(CRServo.Direction.REVERSE);

        //smash = MediaPlayer.create(hardwareMap.appContext, R.raw.smash);
        //running = MediaPlayer.create(hardwareMap.appContext, R.raw.running);
        running.start();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            running.pause();
            smash.start();
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)

            if (gamepad1.left_bumper) {
                slowFactor = 0.35;
            } else {
                slowFactor = 1;
            }

            if(gamepad1.dpad_left){
                leftClamp.setPower(1);
                rightClamp.setPower(1);
            } else if(gamepad1.dpad_right){
                leftClamp.setPower(-1);
                rightClamp.setPower(-1);
            } else {
                leftClamp.setPower(0);
                rightClamp.setPower(0);
            }
            if (gamepad1.dpad_up){
                if (raiserButton.getState()) { // this is negated for some reason
                    raiser.setPower(1);
                    buttonFlag = false; // overrides a

                } else {
                    raiser.setPower(0);
                }

            } else if(gamepad1.dpad_down){
                raiser.setPower(-1);
                buttonFlag = false;

            } else {
                raiser.setPower(0);
            }

            if (gamepad1.left_trigger > 0.001) {
                setRotationMovement();
                leftfrontMotor.setPower(1 * slowFactor);
                rightbackMotor.setPower(1 * slowFactor);

                leftbackMotor.setPower(1 * slowFactor);
                rightfrontMotor.setPower(1 * slowFactor);
            } else if (gamepad1.right_trigger > 0.001) {
                setRotationMovement();
                leftfrontMotor.setPower(-1 * slowFactor);
                rightbackMotor.setPower(-1 * slowFactor);

                leftbackMotor.setPower(-1 * slowFactor);
                rightfrontMotor.setPower(-1 * slowFactor);
            } else {
                setDefaultMovement();
                leftfrontMotor.setPower(-gamepad1.right_stick_y * slowFactor);
                rightbackMotor.setPower(-gamepad1.right_stick_y * slowFactor);

                leftbackMotor.setPower(-gamepad1.left_stick_x * slowFactor);
                rightfrontMotor.setPower(-gamepad1.left_stick_x * slowFactor);
            }

            if (gamepad1.a) {
                if (!buttonFlag) {
                    buttonFlag = true;
                    raiser.setPower(1);
                }
            } else {
                if (buttonFlag) {
                    if (raiserButton.getState()) {
                        raiser.setPower(1);
                    } else {
                        buttonFlag = false;
                    }
                }
            }

            //telemetry.addData("Servo Max: ", leftClamp.MAX_POSITION);
            //telemetry.addData("Servo Min: ", leftClamp.MIN_POSITION);

            //telemetry.addData("Right Position: ", rightClamp.getPower());
            telemetry.addData("Left Position: ", leftClamp.getPower());
            telemetry.addData("Button: ", raiserButton.getState());

            telemetry.update();
        }
    }

    private void setDefaultMovement() {
        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    private void setRotationMovement() {
        leftfrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);
    }
}