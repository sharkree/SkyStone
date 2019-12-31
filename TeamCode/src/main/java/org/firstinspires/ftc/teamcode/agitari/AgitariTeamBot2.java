package org.firstinspires.ftc.teamcode.agitari;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

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
    public static final double MID_SERVO = 0.5 ;
    public static final double HD_HEX_COUNTS_PER_ROTATION = 1120; //  Rev HD Hex motor
    public static final double CORE_HEX_COUNTS_PER_ROTATION = 288; //  Rev Core Hex motor

    public static final double DRIVE_GEAR_REDUCTION  = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 2.953 ;     // 75mm Rev Mecanum Wheels
    public static final double CORE_HEX_COUNTS_PER_INCH =
            (CORE_HEX_COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.075;     // Larger is more responsive, but also less stable

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

    private LinearOpMode opMode;
    private Telemetry telemetry;

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode, HardwareMap ahwMap) {
        this.opMode = opMode;

        this.telemetry = opMode.telemetry;

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        wheelFrontLeft  = hwMap.get(DcMotor.class, "wheel_front_left");
        wheelFrontRight = hwMap.get(DcMotor.class, "wheel_front_right");

        wheelBackLeft  = hwMap.get(DcMotor.class, "wheel_back_left");
        wheelBackRight = hwMap.get(DcMotor.class, "wheel_back_right");

        intakeLeft  = hwMap.get(DcMotor.class, "intake_left");
        intakeRight = hwMap.get(DcMotor.class, "intake_right");

        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Lifter
        linearMotion = hwMap.get(DcMotor.class, "linear_motion");

        // Define and Initialize grabber servo.
        grabber = hwMap.get(Servo.class, "grabber");

        turnTable = hwMap.get(Servo.class, "turn_table");

        clutchLeft = hwMap.get(Servo.class, "clutch_left");
        clutchRight = hwMap.get(Servo.class, "clutch_right");

        // IMU gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hwMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);
    }

    public void setPower(Gamepad gamepad){
        double lx = gamepad.left_stick_x;
        double ly = gamepad.left_stick_y;
        double rx = gamepad.right_stick_x;
        double wheelFrontRightPower = -lx - rx - ly;
        double wheelBackRightPower = lx - rx - ly;
        double wheelFrontLeftPower = -lx - rx + ly;
        double wheelBackLeftPower = lx - rx + ly;

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
        intakeLeft.setPower(0.5);
        intakeRight.setPower(0.5);
    }

    public void revIntake() {
        intakeLeft.setPower(-0.5);
        intakeRight.setPower(-0.5);
    }

    public void stopIntake() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    public void liftUp() {
        linearMotion.setPower(0.85);
    }

    public void liftDown() {
        linearMotion.setPower(-0.425);
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


    //Gyro Stuff for shortening everyone's autonomous
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        wheelBackRight.setPower(0);
        wheelBackLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelFrontLeft.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        wheelFrontLeft.setPower(-leftSpeed);
        wheelFrontRight.setPower(rightSpeed);
        wheelBackLeft.setPower(-leftSpeed);
        wheelBackRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive (double speed, double distance, double angle) {
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * CORE_HEX_COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            int newFrontLeftTarget = wheelFrontLeft.getCurrentPosition() - moveCounts;
            int newFrontRightTarget = wheelFrontRight.getCurrentPosition() + moveCounts;
            wheelFrontLeft.setTargetPosition(newFrontLeftTarget);
            wheelFrontRight.setTargetPosition(newFrontRightTarget);

            int newBackLeftTarget = wheelBackLeft.getCurrentPosition() - moveCounts;
            int newBackRightTarget = wheelBackRight.getCurrentPosition() + moveCounts;
            wheelBackLeft.setTargetPosition(newBackLeftTarget);
            wheelBackRight.setTargetPosition(newBackRightTarget);

            wheelBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            forward(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (wheelFrontLeft.isBusy() ||
                            wheelFrontRight.isBusy() ||
                            wheelBackLeft.isBusy() ||
                            wheelBackRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                wheelFrontLeft.setPower(-leftSpeed);
                wheelFrontRight.setPower(rightSpeed);
                wheelBackLeft.setPower(-leftSpeed);
                wheelBackRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",
                        newFrontLeftTarget,
                        newFrontRightTarget,
                        newBackLeftTarget,
                        newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",
                        wheelFrontLeft.getCurrentPosition(),
                        wheelFrontRight.getCurrentPosition(),
                        wheelBackLeft.getCurrentPosition(),
                        wheelBackRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  -leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            stop();

            // Turn off RUN_TO_POSITION
            wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void forward(double speed) {
        wheelFrontLeft.setPower(-speed);
        wheelFrontRight.setPower(speed);
        wheelBackLeft.setPower(-speed);
        wheelBackRight.setPower(speed);
    }
    private void stop() {
        wheelBackLeft.setPower(0);
        wheelBackLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelFrontLeft.setPower(0);
    }

    public void autoIntake() {
        ElapsedTime holdTimer = new ElapsedTime();
        startIntake();
        // Move forward a little bit
        forward(0.75);
        holdTimer.reset();
        while (opMode.opModeIsActive() && holdTimer.time() < 1) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        stop();
        stopIntake();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && holdTimer.time() < 0.50) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        // Stop all intake;

        startIntake();

        holdTimer.reset();
        while (opMode.opModeIsActive() && holdTimer.time() < 0.50) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        // Stop all intake;
        stopIntake();
    }
}