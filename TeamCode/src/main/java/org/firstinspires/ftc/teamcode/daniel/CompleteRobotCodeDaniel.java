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

package org.firstinspires.ftc.teamcode.daniel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.agitari.AgitariTeamBot.ARM_POWER;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="CompleteRobotCodeDaniel", group="Linear Opmode")
//@Disabled
public class CompleteRobotCodeDaniel extends LinearOpMode {

    public double wheelFrontRightPower = 0;
    public double wheelFrontLeftPower = 0;
    public double wheelBackRightPower = 0;
    public double wheelBackLeftPower = 0;

    public double turbo = 0;

    public DcMotor wheelFrontRight = null;
    public DcMotor wheelFrontLeft = null;
    public DcMotor wheelBackRight = null;
    public DcMotor wheelBackLeft = null;
    public Servo clutch = null;
    public Servo grabber = null;
    public DcMotor arm = null;



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap robot)  {
        wheelBackLeft  = robot.get(DcMotor.class, "wheelBackLeft");
        wheelBackRight = robot.get(DcMotor.class, "wheelBackRight");
        wheelFrontLeft  = robot.get(DcMotor.class, "wheelFrontLeft");
        wheelFrontRight = robot.get(DcMotor.class, "wheelFrontRight");
    }


    @Override

    public void runOpMode() {
        init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            this.turbo = turbo;

            double y1 = gamepad1.left_stick_x;
            double x1 = gamepad1.right_stick_x;
            double x2 = gamepad1.right_trigger;
            wheelFrontRightPower = y1 + x2 - x1;
            wheelBackRightPower = -y1 - x2 - x1;
            wheelFrontLeftPower = y1 - x2 + x1;
            wheelBackLeftPower = -y1 + x2 + x1;

            double max = Math.max(Math.abs(wheelFrontRightPower), Math.max(Math.abs(wheelBackRightPower),
                    Math.max(Math.abs(wheelFrontLeftPower), Math.abs(wheelBackLeftPower))));

            if (max > 1.0) {
                wheelFrontRightPower /= max;
                wheelBackRightPower /= max;
                wheelFrontLeftPower /= max;
                wheelBackLeftPower /= max;
            }

//            wheelFrontRightPower *= turbo;
//            wheelBackRightPower *= turbo;
//            wheelFrontLeftPower *= turbo;
//            wheelBackLeftPower *= turbo;

            wheelFrontRight.setPower(wheelFrontRightPower);
            wheelFrontLeft.setPower(wheelFrontLeftPower);
            wheelBackRight.setPower(wheelBackRightPower);
            wheelBackLeft.setPower(wheelBackLeftPower);

            telemetry.addData("Power wheelFrontRightPower", "Power (%.2f)", wheelFrontRightPower);
            telemetry.addData("Power wheelFrontLeftPower", "Power (%.2f)", wheelFrontLeftPower);
            telemetry.addData("Power wheelBackRightPower", "Power (%.2f)", wheelBackRightPower);
            telemetry.addData("Power wheelBackLeftPower", "Power (%.2f)", wheelBackLeftPower);
            telemetry.update();
        }
    }

    public void forward(double turbo){
        this.turbo = turbo;
        wheelFrontRightPower = 1 * turbo;
        wheelFrontLeftPower = 1 * turbo;
        wheelBackRightPower = 1 * turbo;
        wheelBackLeftPower = 1 * turbo;
    }

    public void backwards(double turbo){
        this.turbo = turbo;
        wheelFrontRightPower = -1 * turbo;
        wheelFrontLeftPower = -1 * turbo;
        wheelBackRightPower = -1 * turbo;
        wheelBackLeftPower = -1 * turbo;
    }

    public void strafeRight(double turbo){
        this.turbo = turbo;
        wheelFrontRightPower = -1 * turbo;
        wheelFrontLeftPower = 1 * turbo;
        wheelBackRightPower = 1 * turbo;
        wheelBackLeftPower = -1 * turbo;
    }

    public void strafeLeft(double speed){
        this.turbo = turbo;
        wheelFrontRightPower = 1 * turbo;
        wheelFrontLeftPower = -1 * turbo;
        wheelBackRightPower = -1 * turbo;
        wheelBackLeftPower = 1 * turbo;

        // Grabber
        if (gamepad1.x) {
            grabber.setPosition(0);
            telemetry.update();
        } else if (gamepad1.y){
            grabber.setPosition(1);
            telemetry.update();
        }

        // Arm control
        double armPower = 0;
        if (gamepad1.left_stick_y != 0.0) {
            armPower = ARM_POWER * gamepad1.left_stick_y;
            armPower = Range.clip(armPower, -1.0, 1.0) ;
            arm.setPower(armPower);
            sleep(50);
        } else if (gamepad1.a)
            arm.setPower (0);

        if (gamepad1.dpad_down)
            clutch.setPosition(1);
        else if (gamepad1.dpad_up)
            clutch.setPosition(0);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Arm", "Power (%.2f)", armPower);
        telemetry.addData("Gamepad", "Stick Y (%.2f)", gamepad1.right_stick_y);
        telemetry.update();

        //sleep(1000);   // optional pause after each move(1000);
    }
}