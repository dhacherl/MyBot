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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.Math;

/**
 * This file provides basic Teleop driving for a robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common hardware class to define the devices on the robot.
 * All device access is managed through the HacherlBot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Mech Drive", group="Hacherl")
// @Disabled
public class Teleop_MechDrive extends OpMode{

    /* Declare OpMode members. */
    HacherlBot robot  = new HacherlBot(); // use the class created to define a HacherlBot's hardware

    /* deadcode
    static class PowerMatrix {
        double flPower, frPower, blPower, brPower;

        PowerMatrix(double argFL, double argFR, double argBL, double argBR) {
            flPower = argFL;
            frPower = argFR;
            blPower = argBL;
            brPower = argBR;
        }

        static PowerMatrix scale(PowerMatrix argIn, double argScale) {
            argIn.flPower = argIn.flPower * argScale;
            argIn.frPower = argIn.frPower * argScale;
            argIn.blPower = argIn.blPower * argScale;
            argIn.brPower = argIn.brPower * argScale;
            return argIn;
        }

        static PowerMatrix add(PowerMatrix argOne, PowerMatrix argTwo) {
            argOne.flPower += argTwo.flPower;
            argOne.frPower += argTwo.frPower;
            argOne.blPower += argTwo.blPower;
            argOne.brPower += argTwo.brPower;
            return argOne;
        }

        static PowerMatrix normalize(PowerMatrix argIn) {
            double maxVal;

            maxVal = Math.max(Math.abs(argIn.flPower),
                              Math.max(Math.abs(argIn.frPower),
                                      Math.max(Math.abs(argIn.blPower),
                                              Math.abs(argIn.brPower))));
            if (maxVal > 1.0) {
                argIn.flPower /= maxVal;
                argIn.frPower /= maxVal;
                argIn.blPower /= maxVal;
                argIn.brPower /= maxVal;
            }
            return argIn;
        }

    }

    PowerMatrix pmForward;
    PowerMatrix pmRight;
    PowerMatrix pmClockwise;
    {
        pmForward  = new PowerMatrix(1.0, 1.0, 1.0, 1.0);
        pmRight = new PowerMatrix(1.0, 1.0, -1.0, -1.0);
        pmClockwise = new PowerMatrix(1.0, -1.0, 1.0, -1.0);
    }
    end of deadcode */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double advance;
        double strafe;
        double rotate;

        // (note: The joystick goes negative when pushed forwards, so negate it)
        advance = -gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        rotate = gamepad1.left_stick_x;

        // adjust sensitivity and dead spot
        advance = HacherlBot.ConditionInput(advance);
        strafe = HacherlBot.ConditionInput(strafe);
        rotate = HacherlBot.ConditionInput(rotate);

        robot.DriveAt(advance, strafe, rotate);

        // Send telemetry message to signify robot running;
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
