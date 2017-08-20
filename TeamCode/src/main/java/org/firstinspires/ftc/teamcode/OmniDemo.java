package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    private Servo leftClamp;
    private Servo rightClamp;
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
        leftClamp = hardwareMap.servo.get("leftClamp");
        rightClamp = hardwareMap.servo.get("rightClamp");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftfrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightfrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftbackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightbackMotor.setDirection(DcMotor.Direction.REVERSE);
        raiser.setDirection(DcMotor.Direction.FORWARD);
        leftClamp.setDirection(Servo.Direction.FORWARD);
        rightClamp.setDirection((Servo.Direction.FORWARD));
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            leftfrontMotor.setPower(-gamepad1.right_stick_y);
            rightbackMotor.setPower(-gamepad1.right_stick_y);

            leftbackMotor.setPower(-gamepad1.right_stick_x);
            rightfrontMotor.setPower(-gamepad1.right_stick_x);
            raiser.setPower(-gamepad1.left_stick_y);
            if(gamepad1.dpad_left){
                leftClamp.setPosition(leftClamp.getPosition()+.1);
                rightClamp.setPosition(rightClamp.getPosition()-.1);
            } else if(gamepad1.dpad_right){
                leftClamp.setPosition(leftClamp.getPosition()-.1);
                rightClamp.setPosition(rightClamp.getPosition()+.1);
            }v
            telemetry.addData("Servo Max: ", leftClamp.MAX_POSITION);
            telemetry.addData("Servo Min: ", leftClamp.MIN_POSITION);


            telemetry.update();
        }
    }
}