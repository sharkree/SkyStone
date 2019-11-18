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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.agitari.AgitariTeamBot;


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

@TeleOp(name="Daniel: Manual Drive Opmode", group="Linear Opmode")
//@Disabled
public class DanielManualOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    AgitariTeamBot robot   = new AgitariTeamBot();   // Use team's bot

    @Override
    public void runOpMode() {
        robot.init (hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;

            double leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
            double rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            // Grabber
            if (gamepad1.x) {
                robot.grabber.setPosition(0);
            } else if (gamepad1.y){
                robot.grabber.setPosition(1);
            }

            // Arm control
            double armPower = 0;
            if (gamepad1.right_stick_y != 0.0) {
                armPower = AgitariTeamBot.ARM_POWER * gamepad1.right_stick_y;
                armPower = Range.clip(armPower, -1.0, 1.0) ;
                robot.arm.setPower(armPower);
                sleep(50);
            } else if (gamepad1.a)
                robot.arm.setPower (0);

            if (gamepad1.dpad_down)
                robot.clutch.setPosition(1);
            else if (gamepad1.dpad_up)
                robot.clutch.setPosition(0);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm", "Power (%.2f)", armPower);
            telemetry.addData("Gamepad", "Stick Y (%.2f)", gamepad1.right_stick_y);
            telemetry.update();

            //sleep(1000);   // optional pause after each move(1000);
        }
    }
}