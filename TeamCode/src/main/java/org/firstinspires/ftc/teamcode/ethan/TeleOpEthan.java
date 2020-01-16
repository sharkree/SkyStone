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

package org.firstinspires.ftc.teamcode.ethan;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.NaHRoboticsTeamBot;




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

@TeleOp(name="NaHRoboticsTeleOpEthan", group="Showcase")
//@Disabled
public class TeleOpEthan extends LinearOpMode {
    public BotCodeEthan bot;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int nextOpenGrabber = 0;
    private int nextTurnTable = 0;
    private int turbo_button = 0;


    @Override
    public void runOpMode()throws InterruptedException  {
        bot = new BotCodeEthan();
        bot.init(this, hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            bot.setPower(gamepad1);

            //Open and Close Clutch to grab foundation
            if (gamepad1.right_trigger>=0.5) {
                bot.openClutch();
            }
            else if (gamepad1.left_trigger>=0.5)   {
                bot.closeClutch();
            }

            //Changing Speed
            if(gamepad2.right_bumper){
                bot.changeTurbo();
                TimeUnit.MILLISECONDS.sleep(200);
            }

            //Intake wheels
            if (gamepad1.x)  {
                bot.startIntake();
            }
            else if (gamepad1.y)  {
               bot.revIntake();
            }
            else {
                bot.stopIntake();
            }

            //lifter
            if (gamepad2.dpad_up)  {
                bot.liftUp();
            }
            else if (gamepad2.dpad_down)  {
                bot.liftDown();
            }
            else  {
                bot.stopLift();
            }

            //grabber

            if (gamepad2.x) {
                nextOpenGrabber++;
                if(nextOpenGrabber % 2 == 0){
                    bot.openGrabber();
                }else {
                   bot.closeGrabber();
                }
                TimeUnit.MILLISECONDS.sleep(500);
            }

            //The Almighty Turn Table
            if (gamepad2.left_bumper) {
                nextTurnTable++;
                if(nextTurnTable % 2 == 0){
                    bot.rotateIn();
                }else {
                    bot.rotateOut();
                }
                TimeUnit.MILLISECONDS.sleep(500);
            }
            //telemetry.addData("Which Bumper", "RightBumper " + gamepad1.right_bumper);
            //telemetry.addData("Which Bumper", "LeftBumper " + gamepad1.left_bumper);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
       }
    }
}