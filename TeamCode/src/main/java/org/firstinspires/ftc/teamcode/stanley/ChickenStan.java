package org.firstinspires.ftc.teamcode.stanley;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="StanleyDance", group="StanBot")
@Disabled
public class ChickenStan extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Send calculated power to wheels
            leftDrive.setPower(1);
            rightDrive.setPower(-1);
            sleep(250);
            for(int i=0; i<5; ++i) {
                leftDrive.setPower(1);
                rightDrive.setPower(1);
                sleep(250);
                leftDrive.setPower(-1);
                rightDrive.setPower(-1);
                sleep(250);
            }
            leftDrive.setPower(-1);
            rightDrive.setPower(1);
            sleep(250);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time stanley: " + runtime.toString());
            telemetry.update();
        }
    }
}
