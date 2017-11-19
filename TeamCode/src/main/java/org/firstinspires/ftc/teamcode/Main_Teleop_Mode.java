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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Pushbot: Teleop Tank", group = "Pushbot")
//@Disabled
public class Main_Teleop_Mode extends OpMode{

    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double clawOffset = 0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.01 ;                 // sets rate to move servo
    //int upTargPos = 1600; //upwards bound of the encoder for the arm
    //int downTargPos = -50; // lower bound of the encoder for the arm
    //static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    //static final double     DRIVE_GEAR_REDUCTION    = 4.0 ;     // This is < 1.0 if geared UP
    //static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    // (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double     DRIVE_SPEED             = 0.6;
    //static final double     TURN_SPEED              = 0.5;

    /*public void driveLine (double speed, double inches){
        int startPos = robot.leftArm.getCurrentPosition();
        robot.leftArm.setPower(speed);
        while (Math.abs(robot.leftArm.getCurrentPosition()-startPos)<=(int)(inches*(1120/4))){

            telemetry.update();

        }
        robot.leftArm.setPower(0);




    }
    */
    /*
    public void linearSlide(double speed, double inches){
        robot.leftArm.setPower(speed);
        robot.leftArm.setTargetPosition((int)inches*1120/4);

        while (robot.leftArm.isBusy()){
            telemetry.update();
        }
        sleep(200);


    }
    */

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("claw",  "Claw Offset = %.2f", clawOffset);
        telemetry.addData("left claw", robot.leftClaw.getPosition());
        telemetry.addData("right claw", robot.rightClaw.getPosition());
        telemetry.addData("Say", "Hello, Driver!");    //
        //robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        telemetry.addData("Encoder Value: ", robot.leftArm.getCurrentPosition());
        double left;
        double right;

        //deadzone
        if (Math.abs(gamepad1.left_stick_y) < 0.125) {
            left = 0;
        } else {
            left = -gamepad1.left_stick_y;
        }
        if (Math.abs(gamepad1.right_stick_y) < 0.125) {
            right = 0;
        } else {
            right = -gamepad1.right_stick_y;
        }


        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper) {
            clawOffset += CLAW_SPEED;
        } else if (gamepad1.left_bumper){
            clawOffset -= CLAW_SPEED;
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.09, 0.09);
        robot.leftClaw.setPosition(robot.LEFT_MID_SERVO + clawOffset);
        robot.rightClaw.setPosition(robot.RIGHT_MID_SERVO - clawOffset);

        // debug info for claws
        telemetry.addData("claw",  "Claw Offset = %.2f", clawOffset);
        telemetry.addData("left claw", robot.leftClaw.getPosition());
        telemetry.addData("right claw", robot.rightClaw.getPosition());

        // Use gamepad buttons to move the arm up (Y) and down (A)
        /*
        if (gamepad1.y) {
            robot.leftArm.setTargetPosition(upTargPos);
            if (robot.leftArm.getCurrentPosition() < robot.leftArm.getTargetPosition())
                robot.leftArm.setPower(robot.ARM_UP_POWER);
            else
                robot.leftArm.setPower(0);
        else if (gamepad1.a) {
            robot.leftArm.setTargetPosition(downTargPos);
            if(robot.leftArm.getCurrentPosition() > robot.leftArm.getTargetPosition())
                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
            else
                robot.leftArm.setPower(0);
            linearSlide(-0.1, -1);
        }

        if (gamepad1.y) {
            linearSlide(0.1, 1);
        }
        else {
            robot.leftArm.setPower(0.0);
        }*/

        if (gamepad1.y) {
            robot.leftArm.setPower(robot.ARM_UP_POWER);
        }
        else if (gamepad1.a) {
            robot.leftArm.setPower(robot.ARM_DOWN_POWER);
        }
        else {
            robot.leftArm.setPower(0.0);
        }
        // Send telemetry message to signify robot running;

        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}