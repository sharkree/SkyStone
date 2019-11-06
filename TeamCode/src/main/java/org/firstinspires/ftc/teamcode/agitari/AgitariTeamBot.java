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
public class AgitariTeamBot
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
    public DcMotor leftDrive   = null;
    public DcMotor rightDrive  = null;
    public Servo clutch = null;
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
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoder to 0
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run using encoders.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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