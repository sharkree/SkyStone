package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

//This is based of of troll robot positioning
//Courtesy of Daniel

public abstract class NaHRoboticsAutonomousSuper extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private final boolean isBlue;

    /* Declare OpMode members. */
    NaHRoboticsTeamBot robot   = new NaHRoboticsTeamBot();   // Use NaH Robotic's team bot
    public NaHRoboticsAutonomousSuper(boolean isBlue) {
        this.isBlue = isBlue;
    }

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = NaHRoboticsTeamBot.VUFORIA_LICENSE_KEY;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**
     * keep track of how far we strafe sideways
     * positive = right
     * negative = left
     */
    private int sidewaysStrafeInches = 0;

    /**
     * How far we went forward
     */
    private int forwardInches = 0;


    @Override
    public void runOpMode() {
        robot.init(this, hardwareMap);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        Orientation angles = robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.addData(">", "Robot Heading = %f", angles.firstAngle);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        /** Wait for the game to begin */
        waitForStart();

        // Drive one foot to improve tensor flow recognition rate
        robot.gyroDrive(1, 12, 0);
        // Keep robot straight
        robot.gyroHold(0.7, 0, 0.5);

        int stepInInches = 6;
        Recognition targetStone = null;
        if (opModeIsActive()) {
            while(true) {
                targetStone = findTarget();
                if (targetStone == null) {
                    telemetry.addData(">", "Can't find a stone target");
                    telemetry.update();
                    break;
                }

                double targetCenter = (targetStone.getTop() + targetStone.getBottom()) / 2;
                telemetry.addData("Target horizontal center: ", "%.02f", targetCenter);
                telemetry.update();
                if (targetCenter < 550) {
                    // Strafe right
                    robot.gyroStrafeSideway(0.7, stepInInches, 0);
                    robot.gyroHold(0.7, 0, 0.5);
                    sidewaysStrafeInches += stepInInches;
                    stepInInches--;
                } else if (targetCenter > 650) {
                    // Strafe left
                    robot.gyroStrafeSideway(0.7, -stepInInches, 0);
                    robot.gyroHold(0.7, 0, 0.5);
                    sidewaysStrafeInches -= stepInInches;
                    stepInInches--;
                } else {
                    break;
                }

                if (stepInInches < 3) {
                    stepInInches = 3;
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        // Drive forward and intake
        if (targetStone != null) {
            telemetry.addData("label", targetStone.getLabel());
            telemetry.addData("  left,top", "%.03f , %.03f",
                    targetStone.getLeft(), targetStone.getTop());
            telemetry.addData("  right,bottom", "%.03f , %.03f",
                    targetStone.getRight(), targetStone.getBottom());
        } else {
            telemetry.addData("label", "Lost track of target stone");
        }
        telemetry.update();

        robot.gyroDrive(1, 12, 0);
        robot.autoIntake();
        int distanceyaxis = isBlue ? -28: -22;
        robot.gyroDrive(1, distanceyaxis, 0);
        int distance = isBlue ? -60 - sidewaysStrafeInches : 60 - sidewaysStrafeInches;
        robot.gyroStrafeSideway(1, distance, 0);
        robot.autoOuttake();
        int distance2 = isBlue ? 21 : -21;
        robot.gyroStrafeSideway(1, distance2, 0);
    }

    private Recognition findTarget() {
        if (tfod == null) {
            return null;
        }

        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions == null) {
            return null;
        }

        Recognition targetSkystone = null;
        double minSkystoneDistance = Double.MAX_VALUE;

        Recognition targetStone = null;
        double minStoneDistance = Double.MAX_VALUE;

        telemetry.addData("# Object Detected", updatedRecognitions.size());

        // step through the list of recognitions and display boundary info.
        int i = 0;
        for (Recognition recognition : updatedRecognitions) {
            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());
            double center = (recognition.getTop() + recognition.getBottom()) / 2;
            double imageCenter = recognition.getImageHeight() / 2;
            double distance = Math.abs(center - imageCenter) / 2;

            if ("Skystone".equals(recognition.getLabel())) {
                if (distance < minSkystoneDistance) {
                    targetSkystone = recognition;
                    minSkystoneDistance = distance;
                }
            } else if ("Stone".equals(recognition.getLabel())) {
                if (distance < minStoneDistance) {
                    targetStone = recognition;
                    minStoneDistance = distance;
                }
            }
        }

        Recognition target = targetSkystone != null ? targetSkystone : targetStone;
        if (target != null) {
            telemetry.addData("Result", "Found target skystone/stone");
            telemetry.addData("  left,top", "%.03f , %.03f",
                    target.getLeft(), target.getTop());
            telemetry.addData("  right,bottom", "%.03f , %.03f",
                    target.getRight(), target.getBottom());
        } else {
            telemetry.addData("Result", "No target stone in sight");
        }
        telemetry.update();
        return target;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

