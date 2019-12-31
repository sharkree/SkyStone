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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.agitari.NaHRoboticsTeamBot;


@TeleOp(name = "StanControllerBop", group = "Showcase Op Mode")
@Disabled
public class StanleyControlled extends LinearOpMode {

    /* Declare OpMode members. */
    NaHRoboticsTeamBot robot = new NaHRoboticsTeamBot();   // Use Agitari's team bot
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
    int left=0;
    int right=0;
    double LinearSlidePower= 0.5;
    double strafeMultiplier= .5;
    double turnMultiplier= .5;
    double strafe;
    double rightFrontPower;
    double rightBackPower;
    double leftFrontPower;
    double leftBackPower;


    public boolean check(double last) {
        boolean x= (last==robot.grabber.getPosition() );
        return x;
    }

    public double capping(double x, double low, double up) {
        x = x < low ? low : x;
        x = x > up ? up: x;
        return x;
    }

    public void initialize(){

        robot.init(this,hardwareMap);
        robot.turnTable.setPosition(1);
        robot.clutchLeft.setPosition(0);
        robot.clutchRight.setPosition(0);

    }

    public void Intake(boolean a){
        if (a){
            robot.startIntake();
        } else {
            robot.stopIntake();
        }
        return;
    }

    public void slowTableForwards(){
        for(double i = 0; i<=1; i+=.1){
            robot.turnTable.setPosition(i);
            sleep(50);
        }
        return;
    }
    public void slowTableBack(){
        for(double i = 1; i>=0; i-=.1){
            robot.turnTable.setPosition(i);
            sleep(50);
        }
        return;
    }
    @Override


    public void runOpMode() {
        initialize();


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
            turn = gamepad1.left_stick_x *turnMultiplier;
            speed = gamepad1.left_stick_y;
            strafe= gamepad1.right_stick_x*strafeMultiplier;

            rightFrontPower= Range.clip(rightFrontPower, -2,2);
            rightBackPower=capping(rightBackPower,-2,2);
            leftFrontPower=capping(leftFrontPower,-2,2);
            leftBackPower=capping(leftBackPower,-2,2);


            rightFrontPower= (speed+turn-strafe);
            rightBackPower= (speed+turn+strafe);
            leftFrontPower= (speed-turn+strafe);
            leftBackPower= (speed-turn-strafe);

            telemetry.addData("turn is", turn);
            telemetry.addData("strafe is", strafe);
            telemetry.addData("speed is", speed);
            telemetry.addData("right front is", rightFrontPower/2);
            telemetry.addData("right back is", rightBackPower/2);
            telemetry.addData("left front is", leftFrontPower/2);
            telemetry.addData("left back is", leftBackPower/2);
            telemetry.update();

            robot.wheelBackLeft.setPower(leftBackPower/2);
            robot.wheelBackRight.setPower(rightBackPower/2);
            robot.wheelFrontRight.setPower(rightFrontPower/2);
            robot.wheelFrontLeft.setPower(leftBackPower/2);
            //He is speed
            robot.wheelBackLeft.getCurrentPosition();



            if(gamepad2.x||gamepad2.y||gamepad2.b||gamepad2.left_trigger==1||gamepad2.right_trigger==1){
                if (gamepad2.x) {
                    xs++;
                    xs = xs % 2;
                    if (xs == 0) {
                        robot.grabber.setPosition(1);
                        } else if (xs== 1){
                        robot.grabber.setPosition(0);
                    }
                }//x is for the grabber
                if (gamepad2.y) {
                    ys++;
                    ys = ys % 2;
                    if (ys == 0) {
                        robot.clutchRight.setPosition(1);
                        robot.clutchLeft.setPosition(0);

                    } else {
                        robot.clutchRight.setPosition(0);
                        robot.clutchLeft.setPosition(1);
                    }
                }//y is for the front servos
                if (gamepad2.b) {
                    bs++;
                    bs = bs % 2;
                    if (bs == 0) {
                        slowTableBack();
                    } else if (bs== 1){

                        slowTableForwards();

                    }//b is for the linear slide servo pos(back or front)
                }
                if (gamepad2.left_trigger==1) {
                    robot.intakeRight.setPower(.3);
                    robot.intakeLeft.setPower(.3);
                }else if (gamepad2.right_trigger==1) {
                    robot.startIntake();
                } else{
                    robot.stopIntake();
                }




                sleep(200);

                //intake
            }





            //for the linear slide
            if(gamepad1.dpad_up){
                robot.linearMotion.setPower(LinearSlidePower);
            } else if(gamepad1.dpad_down){
                robot.linearMotion.setPower(-1*LinearSlidePower);
            } else{
                robot.linearMotion.setPower(0);
            }



            //optional makes right and left bumper make noises
            /**/

            //end of optional
        }
    }
}


/*
    command option l is auto format


 */