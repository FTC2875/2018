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

import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name="PictoAuto", group ="Concept")
//@Disabled
public class PictoAuto extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ARkaptL/////AAAAGZh9qW5VjUFXqr6Ifl7pO9wYYX/eCbWeNSabmx/9Pyp8LKUH2PgfYHhy5ctv9/d2lZRy4L3KjY6lVgWxezb0lJfZFzmGu3Seuxtdo9/PnBvu0AM6yRptIOR3m79S9K78FGG9aroK9d3KS+WcRBIg7WJYSboPDxnjPlwLT9qSaUFLvi4p9LC1X24ITUHE6nUve2aHM4zQ8i2KQ7hUiFQ9R8dUuk8lfjw4E/bcVWfr8vVMNZx8o/jsQPl5QxH2lth52jQw1tbFVixp4zNJ0PvicmDQftXHIWnGag9NBIi5jOJmUBcfFr22EwCnxQJQ7ZkS4ydZe3uhGtrzYSL5ymsN6FnORvcE2GTrF0XaZpa0Saon\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables columnLists = this.vuforia.loadTrackablesFromAsset("Cropped_targets2");
        VuforiaTrackable leftTarget = columnLists.get(2);
        leftTarget.setName("leftTarget");  // Left

        VuforiaTrackable centerTarget  = columnLists.get(1);
        centerTarget.setName("centerTarget");  // Center

        VuforiaTrackable rightTarget  = columnLists.get(0);
        rightTarget.setName("rightTarget");  // Right


        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
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


        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();


        columnLists.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

            if (lastLocation != null) {
                String strPos = format(lastLocation);

                String[] values = finalize(strPos);
                telemetry.addData("Pitch", values[0]);
                telemetry.addData("Yaw", values[1]);
                telemetry.addData("Roll", values[2]);
                telemetry.addData("X", values[3]);
                telemetry.addData("Z", values[4]);
                telemetry.addData("Y", values[5]);


            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }


    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    String [] finalize(String strPos){

        strPos = strPos.replaceAll("[^0-9\\s+.-]", "");
        strPos = strPos.trim();
        strPos = strPos +" ";
        String[] values = strPos.split("\\s+");
        return values;


    }
}
