package org.firstinspires.ftc.teamcode.agitari;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This class defines the team Agitari's robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * Note:  All names are lower case and some have single spaces between words.
 */
public class AgitariTeamBot2
{
    public static final double MID_SERVO = 0.5 ;
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
    public DcMotor wheelFrontLeft = null;//wheels
    public DcMotor wheelFrontRight = null;

    public DcMotor wheelBackLeft = null;
    public DcMotor wheelBackRight = null;

    public DcMotor lifter = null;//linear slide
    public DcMotor intakeRight = null;//intake wheels
    public DcMotor intakeLeft = null;
    public Servo clutch = null;//front clutchers
    public Servo grabber = null;
    public DcMotor arm = null;


    /** REV expansion hub's built-in Gyro sensor. */
    public BNO055IMU imu;

    /* local OpMode members. */
    private HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        wheelFrontLeft  = hwMap.get(DcMotor.class, "wheelFrontLeft");
        wheelFrontRight = hwMap.get(DcMotor.class, "wheelFrontRight");

        wheelBackLeft  = hwMap.get(DcMotor.class, "wheelBackLeft");
        wheelBackRight = hwMap.get(DcMotor.class, "wheelBackRight");

        // Lifter
        lifter = hwMap.get(DcMotor.class, "lifter");

        // Arm: core hex motor
        arm = hwMap.get(DcMotor.class, "arm");

        // Define and Initialize grabber servo.
        grabber = hwMap.get(Servo.class, "grabber");
        grabber.setPosition(MID_SERVO);

        clutch = hwMap.get(Servo.class, "clutch");

        // IMU gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}