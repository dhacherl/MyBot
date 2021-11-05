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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Boolean.FALSE;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *

 */
public class HacherlBot
{
    /* Public OpMode members. */
    public BNO055IMU revIMU;
    public Servo pointerServo;
    public static final double SERVO_HALF_RANGE = 135.0;
    public static final double SERVO_FULL_RANGE = 2.0 * SERVO_HALF_RANGE;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private DcMotor  frontLeftDrive   = null;
    private DcMotor  frontRightDrive  = null;
    private DcMotor  backLeftDrive   = null;
    private DcMotor  backRightDrive  = null;
    private final DcMotor[] driveMotors = new DcMotor[4];
    private double[] motorPower = new double[4];
    boolean bCurrentlyUsingEncoders = true;

    // constants for indices
    private final static int indexFL = 0;
    private final static int indexFR = 1;
    private final static int indexBL = 2;
    private final static int indexBR = 3;
    private final static int maxMotorIndex = 3;
    // Unit vectors in power space for each of the units in motion space
    private final static double[] unitAdvance = {1.0, 1.0, 1.0, 1.0};
    private final static double[] unitStrafe = {1.0, -1.0, -1.0, 1.0};
    private final static double[] unitRotate = {1.0, -1.0, 1.0, -1.0};


    private final static double deadZoneSize = 0.03;   // size of joystick dead zone
    private final static double sensitivityCurve = 2.0; // curvature of sensitivity; 1.0 == linear

    /*
     * Code to take a joystick input and condition it.
     *  - Expand deadzone around stick dead center
     *  - Scale input to reduce sensitivity near center (increasing near full stick)
     */
    static double ConditionInput(double rawInput) {
        double cooked;
        boolean signPositive = rawInput > 0.0;

        cooked = Math.abs(rawInput);
        if (cooked <= deadZoneSize ) {
            cooked = 0.0;
        } else {
            cooked = Math.pow(cooked, sensitivityCurve);
        }
        return signPositive ? cooked : -cooked;
    }



    /* Constructor */
    public HacherlBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hwMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hwMap.get(DcMotor.class, "back_right_drive");

        // Stick the motors in an array for looping purposes
        driveMotors[indexFL] = frontLeftDrive;
        driveMotors[indexFR] = frontRightDrive;
        driveMotors[indexBL] = backLeftDrive;
        driveMotors[indexBR] = backRightDrive;

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        for (int i = 0; i <= maxMotorIndex; i++) {
            // Set all motors to run using encoders.
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Set to zero power, just in case
            // driveMotors[i].setPower(0);
            // If the motors aren't powered, we don't want them turning
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        revIMU = hwMap.get(BNO055IMU.class, "imu");

        pointerServo = hwMap.get(Servo.class, "pointer_servo");
        pointerServo.setPosition(0.5);
    }

    /* Basic Teleop drive method */
    public void DriveAt(double advance, double strafe, double rotate) {
        // translate from control space to power space
        for (int i = 0; i <= maxMotorIndex; i++) {
            motorPower[i] = advance * unitAdvance[i]
                            + strafe * unitStrafe[i]
                            + rotate * unitRotate[i];
        }

        // make sure we're not overpowering motor
        double maxVal = 0.0;
        for (int i = 0; i <= maxMotorIndex; i++) {
            maxVal = Math.max(maxVal, Math.abs(motorPower[i]));
        }
        if (maxVal > 1.0) {
            for (int i = 0; i <= maxMotorIndex; i++) {
                motorPower[i] /= maxVal;
            }
        }

        frontLeftDrive.setPower(motorPower[indexFL]);
        frontRightDrive.setPower(motorPower[indexFR]);
        backLeftDrive.setPower(motorPower[indexBL]);
        backRightDrive.setPower(motorPower[indexBR]);
        // telemetry.addData("fl    fr", " %.3f  %.3f", motorPower[indexFL], motorPower[indexFR]);
        // telemetry.addData("bl    br", " %.3f  %.3f", motorPower[indexBL], motorPower[indexBR]);

    }

    /*
     * Does using encoders during Teleop driving make any difference?
     * Let's be able to change on the fly and see!
     */
    public void useEncoders(boolean bUse) {
        if (bUse != bCurrentlyUsingEncoders) {
            // Desired state has changed!
            DcMotor.RunMode mode = bUse ? DcMotor.RunMode.RUN_USING_ENCODER
                                        : DcMotor.RunMode.RUN_WITHOUT_ENCODER;

            for (DcMotor eachMotor: driveMotors) {
                eachMotor.setMode(mode);
            }
            bCurrentlyUsingEncoders = bUse;
        }
    }

    private static final double TICKS_PER_ADVANCE_CM = 17;  /* totally wrong guess */
    /* Basic autonomous motion method */
    public void AutoAdvance(float advanceCM, double speed, LinearOpMode that) {
        int ticks = (int)(advanceCM * TICKS_PER_ADVANCE_CM);

        if (!that.opModeIsActive()) {
            return;
        }

        // Set target
        for (int i = 0; i <= maxMotorIndex; i++) {
            // Note that the unit vector will reverse the tick sign if necessary
            driveMotors[i].setTargetPosition((int)(ticks * unitAdvance[i]));
        }

        // Prepare to run autonomously
        for (int i = 0; i <= maxMotorIndex; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

         // Go!
        for (int i = 0; i <= maxMotorIndex; i++) {
            driveMotors[i].setPower(speed);
        }

        // Are we there yet?
        runningLoop:
        while (that.opModeIsActive()) {
            for (int i = 0; i <= maxMotorIndex; i++) {
                if (!driveMotors[i].isBusy()) {
                    // At least one motor is done, so stop
                    break runningLoop;
                }
            }
        }

        // Quit moving
        StopAll();
    }

    private static final double TICKS_PER_STRAFE_CM = 18;  /* totally wrong guess */
    /* Basic autonomous motion method */
    public void AutoStrafe(float strafeCM, double speed, LinearOpMode that) {
        int ticks = (int)(strafeCM * TICKS_PER_STRAFE_CM);

        if (!that.opModeIsActive()) {
            return;
        }

        // Set target
        for (int i = 0; i <= maxMotorIndex; i++) {
            // Note that the unit vector will reverse the tick sign if necessary
            driveMotors[i].setTargetPosition((int)(ticks * unitStrafe[i]));
        }

        // Prepare to run autonomously
        for (int i = 0; i <= maxMotorIndex; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Go!
        for (int i = 0; i <= maxMotorIndex; i++) {
            driveMotors[i].setPower(speed);
        }

        // Are we there yet?
        runningLoop:
        while (that.opModeIsActive()) {
            for (int i = 0; i <= maxMotorIndex; i++) {
                if (!driveMotors[i].isBusy()) {
                    // At least one motor is done, so stop
                    break runningLoop;
                }
            }
        }

        // Quit moving
        StopAll();
    }

    /* stop quickly */
    public void StopAll() {
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
 }

