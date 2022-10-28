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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// import static org.firstinspires.ftc.teamcode.HacherlBot.SERVO_FULL_RANGE;
// import static org.firstinspires.ftc.teamcode.HacherlBot.SERVO_HALF_RANGE;

/**
 * This file provides basic Teleop driving for a robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common hardware class to define the devices on the robot.
 * All device access is managed through the HacherlBot class.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Absolute Drive", group="Hacherl")
// @Disabled
public class Absolute_Drive extends OpMode{

    /* Declare OpMode members. */
    HacherlBot robot  = new HacherlBot(); // use the class created to define a HacherlBot's hardware
    double initialHeading;
    double lastCommandedHeading;
    boolean intakeMode = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //BUGBUG Initialize IMU, determine starting robot orientation
        BNO055IMU.Parameters imuParams;
        imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        robot.revIMU.initialize(imuParams);
        // Orientation
        initialHeading = robot.revIMU.getAngularOrientation().firstAngle;
        lastCommandedHeading = initialHeading;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Hello Driver, initial orientation is", "%.1f", initialHeading);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Helper function to keep angles within (-180,180)
     */
    double normalizeAngle(double angle) {
        double newAngle = angle;
        while (newAngle < -180) newAngle += 360;
        while (newAngle > 180) newAngle -= 360;
        return newAngle;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double advance;
        double strafe;
        double rotate;
        double upMotion;
        double overMotion;
        double currentHeadingDelta;
        Orientation facing;

        // (note: The joystick goes negative when pushed forwards, so negate it)
        upMotion = -gamepad1.right_stick_y;
        overMotion = gamepad1.right_stick_x;
        rotate = gamepad1.left_stick_x;

        // adjust for sensitivity and dead spot
        upMotion = HacherlBot.ConditionInput(upMotion);
        overMotion = HacherlBot.ConditionInput(overMotion);
        rotate = HacherlBot.ConditionInput(rotate);

        facing = robot.revIMU.getAngularOrientation();
        if (gamepad1.right_stick_button) {
            // orientation reset requested
            initialHeading = facing.firstAngle;
        }

        // The robot has special driving mode when the intake is enabled.
        if (gamepad1.left_stick_button) {
            if (!intakeMode) {
                // If we're just switching in to intake mode, turn on the intake
                robot.IntakeOn();
                intakeMode = true;
            }
        } else {
            if (intakeMode) {
                // If we're just switching out of intake mode, turn off the intake
                robot.IntakeOff();
                intakeMode = false;
            }
        }

        // Translate from up and over to advance and strafe
        // a positive currentHeadingDelta indicates that the robot has rotated COUNTER-clockwise
        // from its initial position (i.e., from the field reference frame), meaning that advance motion
        // needs to be slightly reduced from the intended up motion, and that there needs to be a
        // small right strafe component added.  At zero angle cos(delta) is 1, and sin(delta) is
        // 0, so the advance is exactly what it was with no correction and strafe remains at 0.
        // With a small angle cos() is just under 1, so advance is just less than upMotion,
        // while (since overMotion is 0) strafe is set to a value just above 0.
        currentHeadingDelta = normalizeAngle(facing.firstAngle - initialHeading);
        double radCHD = Math.toRadians(currentHeadingDelta);
        advance = upMotion*Math.cos(radCHD) - overMotion*Math.sin(radCHD);
        strafe = overMotion*Math.cos(radCHD) + upMotion*Math.sin(radCHD);

        // if the servo is capable of pointing that far off center, point it to "up"
        // if ((currentHeadingDelta > -SERVO_HALF_RANGE) && (currentHeadingDelta < SERVO_HALF_RANGE)) {
        //     robot.pointerServo.setPosition(0.5 + currentHeadingDelta/(SERVO_FULL_RANGE));
        // }

        if (intakeMode) {
            // If we're in intake mode then point the front of the robot in the direction of motion
            // so that objects we encounter will hit the intake mechanism.
            // We do this by treating the overMotion and upMotion as a velocity vector in
            // Cartesian coordinates which we then convert to polar coordinates (r, theta).
            // We use the r to see if we're moving enough to justify rotation, and the theta to
            // set the desired orientation.
            double rMotion = Math.sqrt(overMotion*overMotion + upMotion*upMotion);
            double thetaMotion = Math.toDegrees(Math.atan2(upMotion, overMotion));
            if (rMotion >= 0.02) {
                // If we're moving, set the heading to maintain to the power direction. Note that
                // thetaMotion is in polar coordinates, with 0 degrees being pure right, while
                // heading is deviation from straight up, which is 90 degrees different.
                lastCommandedHeading = thetaMotion - 90.0;
            }
            rotate = 0.0; // ignore any commanded rotation input
        }
        else { //not in intake steering mode
            // Use the d-pad to indicate desired absolute orientation
            if (gamepad1.dpad_up) {
                if (gamepad1.dpad_left) {
                    // up and left
                    lastCommandedHeading = initialHeading + 45;
                } else if (gamepad1.dpad_right) {
                    // up and right
                    lastCommandedHeading = initialHeading - 45;
                } else {
                    // just up
                    lastCommandedHeading = initialHeading;
                }
            } else if (gamepad1.dpad_down) {
                if (gamepad1.dpad_left) {
                    // down and left
                    lastCommandedHeading = initialHeading + 135;
                } else if (gamepad1.dpad_right) {
                    // down and right
                    lastCommandedHeading = initialHeading - 135;
                } else {
                    // just down
                    lastCommandedHeading = initialHeading - 180;
                }
            } else if (gamepad1.dpad_left) {
                // pure left
                lastCommandedHeading = initialHeading + 90;
            } else if (gamepad1.dpad_right) {
                // pure right
                lastCommandedHeading = initialHeading - 90;
            }
        }
        lastCommandedHeading = normalizeAngle(lastCommandedHeading);


        // Maintain our last heading, unless we're being commanded to turn
        if (rotate != 0.0) {
            // We're being commanded to turn to a new heading, so remember where we are now, so that
            // we will stay here in the future.
            lastCommandedHeading = facing.firstAngle;
        } else {
            // We're not being told to turn, so we're supposed to be holding the current orientation.
            // If we're not doing so, add some rotation to get back to where we should be.
            double headingDrift =  normalizeAngle(facing.firstAngle - lastCommandedHeading);

            // Note that (conveniently!) headingDrift is positive if we rotated counterclockwise
            // from our desired orientation, but that the rotate argument to DriveAt indicates
            // the amount of clockwise rotation desired.  Thus the signs work out correctly and
            // all we need do is scale the amount of rotation desired by the amount of drift
            // observed.

            // This is an arbitrary scale factor, but seems to work.
            rotate = 0.01 * headingDrift;
        }

       robot.DriveAt(advance, strafe, rotate);

        // Send telemetry message to signify robot running;
        telemetry.addData("facing", " %.1f %.1f %.1f", facing.firstAngle, facing.secondAngle, facing.thirdAngle);
        telemetry.addData("heading delta", "%.1f", currentHeadingDelta);
        telemetry.addData("up and over", "%.3f   %.3f", upMotion, overMotion);
        telemetry.addData("advance",  "%.3f", advance);
        telemetry.addData("strafe",  "%.3f", strafe);
        telemetry.addData("rotate", "%.3f", rotate);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.StopAll();
    }
}
