package org.firstinspires.ftc.teamcode.agitari;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.agitari.AgitariTeamBot.ARM_POWER;


/**
 * This class defines the team Agitari's robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * Note:  All names are lower case and some have single spaces between words.
 */
public class AgitariTeamBot2
{
    public static final double MID_SERVO = 0.5;
    public static final double INIT_SERVO = 1;
    public static final double HD_HEX_COUNTS_PER_ROTATION = 1120; //  Rev HD Hex motor

    public static final double DRIVE_GEAR_REDUCTION  = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 3.543 ;     // For figuring circumference
    public static final double HD_HEX_COUNTS_PER_INCH =
            (HD_HEX_COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double CORE_HEX_COUNTS_PER_ROTATION = 288; //  Rev Core Hex motor

    public static final String VUFORIA_LICENSE_KEY = "AXIXrHj/////AAABmeEMqruXWUCBuoatjfPPvO" +
            "Qv4U/tRYoBqMMvXyAoHLHWYYYQSPx3ZOZ7GcdOCuTHK5HYM6oJ4gX1ZTxVec9RI4xa5ZOSPgTQvSo" +
            "Er2GJeRPohMXHEy6DRer3JDhvcPN32CzBiKJf2i60dFivASvEyU2EGRHGKq41VjsOk09o2q0Wr9ly" +
            "oEzdhNjMgAf8OfPn8wl93IM0Bo2+hH0ZtUSmZUoyBu53qlB0wgZ+FJYHxOOXdhim0ka+qa0CkFOkn" +
            "lN35bbLE6yNSyBOV86FaSZ0UuBNXfCX4O0IWh7qSBXcU/cQVMw3faOu8Hx3LiReY1lcQ1I4q0QP05" +
            "IUr5l71eQEMFLO71ByBWG95IkHucF5iyrA";

    public static final double ARM_POWER = -0.75 ;

    /* Public OpMode members. */
    public DcMotor wheelFrontLeft = null;
    public DcMotor wheelFrontRight = null;

    public DcMotor wheelBackLeft = null;
    public DcMotor wheelBackRight = null;

    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;

    public DcMotor linearMotion = null;

    public Servo clutchLeft = null;
    public Servo clutchRight = null;
    public Servo grabber = null;
    public Servo turnTable = null;

    /** REV expansion hub's built-in Gyro sensor. */
    public BNO055IMU imu;

    /* local OpMode members. */
    private HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    private double turbo;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        wheelFrontLeft  = hwMap.get(DcMotor.class, "wheel_front_left");
        wheelFrontRight = hwMap.get(DcMotor.class, "wheel_front_right");

        wheelBackLeft  = hwMap.get(DcMotor.class, "wheel_back_left");
        wheelBackRight = hwMap.get(DcMotor.class, "wheel_back_right");

        intakeLeft  = hwMap.get(DcMotor.class, "intake_left");
        intakeRight = hwMap.get(DcMotor.class, "intake_right");

        //intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Lifter
        linearMotion = hwMap.get(DcMotor.class, "linear_motion");

        // Define and Initialize grabber servo.
        grabber = hwMap.get(Servo.class, "grabber");
        grabber.setPosition(INIT_SERVO);

        turnTable = hwMap.get(Servo.class, "turn_table");
        turnTable.setPosition(INIT_SERVO);

        clutchLeft = hwMap.get(Servo.class, "clutch_left");
        clutchRight = hwMap.get(Servo.class, "clutch_right");

        // IMU gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void setPower(Gamepad gamepad, Telemetry telemetry){
        double lx = gamepad.left_stick_x;
        double ly = gamepad.left_stick_y;
        double rx = gamepad.right_stick_x;
        double wheelFrontRightPower = 0.5 * (-lx - rx - ly);
        double wheelBackRightPower = 0.5 * (lx - rx - ly);
        double wheelFrontLeftPower = 0.5 * (-lx - rx + ly);
        double wheelBackLeftPower = 0.5 * (lx - rx + ly);

        double max = Math.max(Math.abs(wheelFrontRightPower), Math.max(Math.abs(wheelBackRightPower),
                Math.max(Math.abs(wheelFrontLeftPower), Math.abs(wheelBackLeftPower))));

        if (max > 1.0) {
            wheelFrontRightPower /= max;
            wheelBackRightPower /= max;
            wheelFrontLeftPower /= max;
            wheelBackLeftPower /= max;
        }

        wheelFrontRight.setPower(wheelFrontRightPower);
        wheelBackRight.setPower(wheelBackRightPower);
        wheelFrontLeft.setPower(wheelFrontLeftPower);
        wheelBackLeft.setPower(wheelBackLeftPower);

        telemetry.addData("Power wheelFrontRightPower", "Power (%.2f)", wheelFrontRightPower);
        telemetry.addData("Power wheelFrontLeftPower", "Power (%.2f)", wheelFrontLeftPower);
        telemetry.addData("Power wheelBackRightPower", "Power (%.2f)", wheelBackRightPower);
        telemetry.addData("Power wheelBackLeftPower", "Power (%.2f)", wheelBackLeftPower);
    }

    public void openClutch() {
        clutchLeft.setPosition(1);
        clutchRight.setPosition(0);
    }

    public void closeClutch() {
        clutchLeft.setPosition(0);
        clutchRight.setPosition(1);
    }

    public void startIntake() {
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setPower(0.5);
        intakeRight.setPower(0.5);
    }

    public void revIntake() {
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setPower(0.5);
        intakeRight.setPower(0.5);
    }

    public void stopIntake() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    public void liftUp() {
        linearMotion.setPower(0.6);
    }

    public void liftDown() {
        linearMotion.setPower(-0.3);
    }

    public void stopLift() {
        linearMotion.setPower(0);
    }

    public void openGrabber() {
        grabber.setPosition(1);
    }

    public void closeGrabber() {
        grabber.setPosition(0);
    }

    public void rotateIn() {
        turnTable.setPosition(1);
    }

    public void rotateOut() {
        turnTable.setPosition(0);
    }
}