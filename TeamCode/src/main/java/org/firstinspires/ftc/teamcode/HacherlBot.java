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
//import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;

//import static java.lang.Boolean.FALSE;

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
 // removed   public Servo pointerServo;
 //   public static final double SERVO_HALF_RANGE = 135.0;
 //   public static final double SERVO_FULL_RANGE = 2.0 * SERVO_HALF_RANGE;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
 //   private ElapsedTime period  = new ElapsedTime();
    private DcMotorEx  frontLeftDrive   = null;
    private DcMotorEx  frontRightDrive  = null;
    private DcMotorEx  backLeftDrive   = null;
    private DcMotorEx  backRightDrive  = null;
    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final double[] motorPower = new double[4];
    // 28 ticks/rev * 6000 rpm * 1 min / 60 seconds = 2800 ticks/sec maximum
    private final static double maxMotorTickRate = 1400;  // slow down!
    private DcMotorEx liftMotor = null;
    DigitalChannel liftBottomTouch = null;
    Servo gripper = null;

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

    // Lift stop information
    // Initial measurements based on 20:1 gearbox
    //private final static int[] liftStops = {0, 136, 736, 1293, 1500};
    private final static int[] liftStops = {0, 800, 4000, 6900, 7845};
    private final static int maxLiftStop = liftStops.length - 1;
    public int curLiftStop = 0;

    private final static double deadZoneSize = 0.03;   // size of joystick dead zone
    private final static double sensitivityCurve = 2.5; // curvature of sensitivity; 1.0 == linear

    /*
     * Code to take a joystick input and condition it.
     *  - Expand deadzone around stick dead center
     *  - Scale input to reduce sensitivity near center (increasing near full stick)
     */
    static double conditionInput(double rawInput) {
        double cooked = Math.abs(rawInput);

        if (cooked <= deadZoneSize ) {
            cooked = 0.0;
        } else {
            cooked = Math.pow(cooked, sensitivityCurve);
        }
        return cooked * Math.signum(rawInput);
    }



    /* Constructor */
    public HacherlBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "front_right_drive");
        backLeftDrive  = hwMap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = hwMap.get(DcMotorEx.class, "back_right_drive");

        // Stick the motors in an array for looping purposes
        driveMotors[indexFL] = frontLeftDrive;
        driveMotors[indexFR] = frontRightDrive;
        driveMotors[indexBL] = backLeftDrive;
        driveMotors[indexBR] = backRightDrive;

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        for (int i = 0; i <= maxMotorIndex; i++) {
            // Set all motors to run using encoders.
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Set to zero power, just in case
            // driveMotors[i].setPower(0);
            // If the motors aren't powered, we don't want them turning
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        revIMU = hwMap.get(BNO055IMU.class, "imu");

        // init the gripper and make sure it's open
        gripper = hwMap.get(Servo.class, "gripper");
        release();

        // init the lift motor
        liftMotor = hwMap.get(DcMotorEx.class, "lift_motor");
        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // init the lift at bottom sensor
        liftBottomTouch = hwMap.get(DigitalChannel.class, "lift_bottom_touch");
        liftBottomTouch.setMode(DigitalChannel.Mode.INPUT);

        // Position the lift just off the bottom
        if (liftBottomTouch.getState()) {
            // Dang it, we're just starting, but the lift is not all the way down.
            // Slowly lower the lift until the sensor trips.
            liftMotor.setVelocity(-200);
            while (liftBottomTouch.getState()) {
                try {
                    Thread.sleep(1);
                }catch (Exception e) {
                    // I'm ignoring this
                }
            }
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // OK, we should now be sitting with the lift at the bottom.  We will let it creep up
        // until the bottom sensor goes off.
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setVelocity(20);
        while (!liftBottomTouch.getState()) {
            try {
                Thread.sleep(1);
            }catch (Exception e) {
                // I'm ignoring this
            }
        }
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curLiftStop = 0;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPositionTolerance(5);
        liftMotor.setTargetPosition(0);
        liftMotor.setVelocity((liftStops[maxLiftStop]/2.0));  // full lift in 2 seconds?

    }

    // Raise the lift one step
    public int liftUp() {
        if (curLiftStop < maxLiftStop) {
            ++curLiftStop;
            liftMotor.setTargetPosition(liftStops[curLiftStop]);
        }
        return curLiftStop;
    }

    // Lower the lift one step
    public int liftDown() {
        if (curLiftStop > 0) {
            --curLiftStop;
            liftMotor.setTargetPosition(liftStops[curLiftStop]);
        }
        return curLiftStop;
    }

    public boolean isLiftMoving() {
        return liftMotor.isBusy();
    }

    //TODO remove this method.  Only needed for initial debugging?
    boolean liftZeroed = false;
    /*
           if (!liftZeroed && liftBottomTouch.getState()) {
            //First time the button has not been pressed
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftZeroed = true;
        }
       */
    public int getLiftPos() {
        return liftMotor.getCurrentPosition();
    }

    /* Basic Teleop drive method */
    public void driveAt(double advance, double strafe, double rotate) {
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

        /* Changed from power to velocity
        frontLeftDrive.setPower(motorPower[indexFL]);
        frontRightDrive.setPower(motorPower[indexFR]);
        backLeftDrive.setPower(motorPower[indexBL]);
        backRightDrive.setPower(motorPower[indexBR]);
         */
        frontLeftDrive.setVelocity(motorPower[indexFL]*maxMotorTickRate);
        frontRightDrive.setVelocity(motorPower[indexFR]*maxMotorTickRate);
        backLeftDrive.setVelocity(motorPower[indexBL]*maxMotorTickRate);
        backRightDrive.setVelocity(motorPower[indexBR]*maxMotorTickRate);

        // telemetry.addData("fl    fr", " %.3f  %.3f", motorPower[indexFL], motorPower[indexFR]);
        // telemetry.addData("bl    br", " %.3f  %.3f", motorPower[indexBL], motorPower[indexBR]);

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

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        liftMotor.setPower(0.0);

    }

    public void grip() {gripper.setPosition(0.0);}
    public void release() {gripper.setPosition(1.0);}

}

