/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.stanley;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.agitari.AgitariTeamBot;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="StanBop", group="Showcase Op Mode")
//@Disabled
public class StanBop extends LinearOpMode {

    /* Declare OpMode members. */
    AgitariTeamBot robot = new AgitariTeamBot();   // Use Agitari's team bot
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants def ine the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    double turn = 0.0;
    double speed = 0.0;
    int xs =0;
    int ys = 0;



    @Override


    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */

        robot.init(hardwareMap);

        int duck = hardwareMap.appContext.getResources().getIdentifier(
                "duck", "raw", hardwareMap.appContext.getPackageName());
        int goose   = hardwareMap.appContext.getResources().getIdentifier(
                "fair",   "raw", hardwareMap.appContext.getPackageName());
//        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        waitForStart();
        while (opModeIsActive()) {
            turn=gamepad1.right_stick_x;
            speed=-1*gamepad1.left_stick_y;
            robot.leftDrive.setPower(speed*3/4+turn/2);
            robot.rightDrive.setPower(speed*3/4-turn/2);
            //He is speed
            if (gamepad1.x) {
                xs++;
            }
            xs=xs%2;
            if (xs==0){
                //robot.frontGrabber.setPosition(1);
                robot.grabber.setPosition(0);
            } else{
                //robot.frontGrabber.setPosition(.5);
                robot.grabber.setPosition(1);

            }
            sleep (200);
            if (gamepad1.y) {
                ys++;
            }
            ys=ys%2;
            if (ys==0){
                robot.clutch.setPosition(1);

            } else{
                robot.clutch.setPosition(0);

            }

            if (gamepad1.dpad_up) {
                robot.Arm.setPower(.65);
            }
            if (gamepad1.dpad_down) {
                robot.Arm.setPower(-.7);
            }
            if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                robot.Arm.setPower(-.1);
            }
            if(gamepad1.dpad_down && gamepad1.dpad_right && gamepad1.dpad_left && gamepad1.dpad_up){
                return;
            }


            // Determine if sound resources are found.
            // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
            if (gamepad1.b) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, duck);
            }


            if (gamepad1.a) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, duck);

            }


        }
    }
}
