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


import org.firstinspires.ftc.teamcode.agitari.AgitariTeamBot2;


@TeleOp(name = "StanControllerBop", group = "Showcase Op Mode")
//@Disabled
public class StanleyControlled extends LinearOpMode {

    /* Declare OpMode members. */
    AgitariTeamBot2 robot = new AgitariTeamBot2();   // Use Agitari's team bot
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
    int xs = 0;
    int ys = 0;
    int bs = 0;
    int as = 0;
    double LinearSlidePower= 0.1;
    double grabberFront = 1;
    double grabberBack=0;
    double strafeMultiplier= 2/5;
    double turnMultiplier= 1/2;
    double strafe;
    double rightFrontPower;
    double rightBackPower;
    double LeftFrontPower;
    double leftBackPower;
    double intakePower=0.3;



    public boolean check(double last) {
        boolean x= (last==robot.grabber.getPosition() );
        return x;
    }



    @Override


    public void runOpMode() {

        robot.init(hardwareMap);

        /*int duck = hardwareMap.appContext.getResources().getIdentifier(
                "duck", "raw", hardwareMap.appContext.getPackageName());
        int goose = hardwareMap.appContext.getResources().getIdentifier(
                "goose", "raw", hardwareMap.appContext.getPackageName());
        int fair = hardwareMap.appContext.getResources().getIdentifier(
                "fair", "raw", hardwareMap.appContext.getPackageName());

         */
//        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        waitForStart();
        while (opModeIsActive()) {

            // start of wheel stuff
            turn = gamepad1.left_stick_x*turnMultiplier;
            speed = 5 * gamepad1.left_stick_y;
            strafe= gamepad1.right_stick_x*strafeMultiplier;

            rightFrontPower= speed+turn+strafe;
            rightBackPower=speed+turn-strafe;
            LeftFrontPower=speed-turn-strafe;
            leftBackPower=speed-turn+strafe;

            robot.wheelBackLeft.setPower(leftBackPower);
            robot.wheelBackRight.setPower(rightBackPower);
            robot.wheelFrontRight.setPower(rightFrontPower);
            robot.wheelFrontLeft.setPower(leftBackPower);
            //He is speed

            if(gamepad1.x||gamepad1.y||gamepad1.b){
                if (gamepad1.x) {
                    xs++;
                    xs = xs % 2;
                    if (xs == 0) {
                        robot.grabber.setPosition(1);
                    } else if (xs== 1){
                        robot.grabber.setPosition(0);
                    }
                }//x is for the grabber
                if (gamepad1.y) {
                    ys++;
                    ys = ys % 2;
                    if (ys == 0) {
                        robot.clutchRight.setPosition(1);
                        robot.clutchLeft.setPosition(1);

                    } else {
                        robot.clutchRight.setPosition(0);
                        robot.clutchLeft.setPosition(0);
                    }
                }//y is for the front servos
                if (gamepad1.b) {
                    bs++;
                    bs = bs % 2;
                    if (bs == 0) {
                        robot.grabber.setPosition(grabberBack);
                    } else if (bs== 1){
                        robot.grabber.setPosition(grabberFront);
                    }//b is for the linear slide servo pos(back or front)
                }
                if (gamepad1.a) {
                    as++;
                    as = as % 2;
                    if (as == 0) {
                        robot.intakeLeft.setPower(intakePower);
                        robot.intakeLeft.setPower(-intakePower);
                    } else if (as== 1){
                        robot.intakeLeft.setPower(0);
                        robot.intakeRight.setPower(0);
                    }//intake wheels
                }
                sleep(150);
            }

            if(gamepad1.dpad_up){
                robot.linearMotion.setPower(LinearSlidePower);
            } else if(gamepad1.dpad_up){
                robot.linearMotion.setPower(-1*LinearSlidePower);
            } else{
                robot.linearMotion.setPower(0);
            }



            //optional makes right and left bumper make noises
            /*if (gamepad1.right_bumper) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, duck);
            }
            if (gamepad1.left_bumper) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goose);

            }
            */

            //end of optional
        }
    }
}


/*
    command option l is auto format


 */