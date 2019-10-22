package org.firstinspires.ftc.teamcode.hongbing;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This op mode is an example showcasing REV expansion hub's build-in gyro sensor
 */
@TeleOp(name="Hongbing: Gyro", group="Showcase Op Mode")
//@Disabled
public class RevGyroShowcase extends LinearOpMode {
    private BNO055IMU imu;

    private Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading: ", angles.firstAngle);
            telemetry.addData("Roll: ", angles.secondAngle);
            telemetry.addData("Pitch: ", angles.thirdAngle);
            telemetry.update();
        }
    }
}
