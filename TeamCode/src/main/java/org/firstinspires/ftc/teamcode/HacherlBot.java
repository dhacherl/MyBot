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
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private DcMotor  frontLeftDrive   = null;
    private DcMotor  frontRightDrive  = null;
    private DcMotor  backLeftDrive   = null;
    private DcMotor  backRightDrive  = null;
    private double[] motorPower = new double[4];

    // constants for indices
    private final static int indexFL = 0;
    private final static int indexFR = 1;
    private final static int indexBL = 2;
    private final static int indexBR = 3;
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

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power, just in case
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        revIMU = hwMap.get(BNO055IMU.class, "imu");

    }

    /* Basic Teleop drive method */
    public void DriveAt(double advance, double strafe, double rotate) {
        // translate from control space to power space
        for (int i = 0; i < 4; i++) {
            motorPower[i] = advance * unitAdvance[i];
            motorPower[i] += strafe * unitStrafe[i];
            motorPower[i] += rotate * unitRotate[i];
        }

        // make sure we're not overpowering motor
        double maxVal = 0.0;
        for (int i = 0; i < 4; i++) {
            maxVal = Math.max(maxVal, Math.abs(motorPower[i]));
        }
        if (maxVal > 1.0) {
            for (int i = 0; i < 4; i++) {
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

