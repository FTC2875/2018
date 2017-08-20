package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);
        raiser.setDirection(DcMotor.Direction.FORWARD);
        leftClamp.setDirection(CRServo.Direction.FORWARD);
        rightClamp.setDirection(CRServo.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            leftfrontMotor.setPower(-gamepad1.right_stick_y);
            rightbackMotor.setPower(-gamepad1.right_stick_y);

            leftbackMotor.setPower(-gamepad1.left_stick_x);
            rightfrontMotor.setPower(-gamepad1.left_stick_x);

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
                raiser.setPower(1);
            } else if(gamepad1.dpad_down){
                raiser.setPower(-1);
            } else {
                raiser.setPower(0);
            }
            //telemetry.addData("Servo Max: ", leftClamp.MAX_POSITION);
            //telemetry.addData("Servo Min: ", leftClamp.MIN_POSITION);

            //telemetry.addData("Right Position: ", rightClamp.getPower());
            telemetry.addData("Left Position: ", leftClamp.getPower());

            telemetry.update();
        }
    }
}