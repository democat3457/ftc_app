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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the Drivebase class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Three Wheels Tank", group = "TeleOp")
//@Disabled
public class Alt_Teleop_Mode_Tank extends OpMode {

    //int upTargPos = 1600; //upwards bound of the encoder for the arm
    //int downTargPos = -50; // lower bound of the encoder for the arm
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 4.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    final double CLAW_SPEED = 0.01;                 // sets rate to move servo
    /* Declare OpMode members. */
    Drivebase robot = new Drivebase(); // use the class created to define a Pushbot's hardware
    // could also use DrivebaseMatrix class.
    double clawOffset = 0;                  // Servo mid position
    private ElapsedTime runtime = new ElapsedTime();

    /*
    public void encoderArm (double speed, double inches, double timeoutS){
        int newArmTarget;

        newArmTarget = robot.getLeftArm().getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        robot.getLeftArm().setTargetPosition(newArmTarget);

        robot.getLeftArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.getLeftArm().setPower(Math.abs(speed));

        while ((runtime.seconds() < timeoutS) && robot.getLeftArm().isBusy()){
            telemetry.addData("Path1", "Running to %7d", newArmTarget);
            telemetry.addData("Path2", "Running at %7d", robot.getLeftArm().getCurrentPosition());
        }

        robot.getLeftArm().setPower(0);

        
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
        robot.getLeftDrive().setMode(Constants.MOTOR_RUN_MODE);
        robot.getRightDrive().setMode(Constants.MOTOR_RUN_MODE);

        // Set Jewel Arm to be upright.
        robot.getJewelArm().setPosition(Constants.JEWEL_UP_LIMIT);
        robot.getColorSensor().enableLed(false);
        robot.getColorSensor().close();

        // debug info for claw
        telemetry.addData("claw", "Claw Offset = %.2f", clawOffset);
        telemetry.addData("left claw", robot.getLeftClaw().getPosition());
        telemetry.addData("right claw", robot.getRightClaw().getPosition());
        telemetry.addData("Jewel Arm Pos", robot.getJewelArm().getPosition());
        telemetry.addData("Say", "Hello, Driver!");

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

        double lr = 0;

        /*if (robot.getTouchSensor().isPressed()){
            robot.resetEncoders(robot.getLeftArm(), DcMotor.RunMode.RUN_USING_ENCODER);
            
            robot.getLeftArm().setPower(0);
        }*/

        if (gamepad1.dpad_left) {
            lr = 1;
        } else if (gamepad1.dpad_right) {
            lr = -1;
        } else if (gamepad1.left_trigger != 0) {
            lr = gamepad1.left_trigger;
        } else if (gamepad1.right_trigger != 0) {
            lr = -gamepad1.right_trigger;
        }

        // chassis movement
        robot.getLeftDrive().setPower(-gamepad1.left_stick_y);
        robot.getRightDrive().setPower(-gamepad1.right_stick_y);
        robot.thirdWheel.setPower(lr);

        /*if (gamepad1.x){
            robot.getLeftDrive().setPower(-1);
            robot.getRightDrive().setPower(1);
        }
        if (gamepad1.b){
            robot.getLeftDrive().setPower(1);
            robot.getRightDrive().setPower(-1);
        } */


        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.right_bumper) {
            clawOffset += CLAW_SPEED;
        } else if (gamepad2.left_bumper) {
            clawOffset -= CLAW_SPEED;
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -Constants.MAX_CLAW_OFFSET, Constants.MAX_CLAW_OFFSET);
        robot.getLeftClaw().setPosition(Constants.LEFT_MID_SERVO + clawOffset);
        robot.getRightClaw().setPosition(Constants.RIGHT_MID_SERVO - clawOffset);

        // debug info for claws
        telemetry.addData("claw", "Claw Offset = %.2f", clawOffset);
        telemetry.addData("left claw", robot.getLeftClaw().getPosition());
        telemetry.addData("right claw", robot.getRightClaw().getPosition());
        telemetry.update();

        // Use gamepad buttons to move the arm up (Y) and down (A)

        if (gamepad2.y) {
            robot.armUp();
            telemetry.addData("getLeftArm() position", robot.getLeftArm().getPosition());
            telemetry.addData("getRightArm() position", robot.getRightArm().getPosition());
            telemetry.update();
        } else if (gamepad2.dpad_up) {
            robot.armUp();
            telemetry.addData("getLeftArm() position", robot.getLeftArm().getPosition());
            telemetry.addData("getRightArm() position", robot.getRightArm().getPosition());
            telemetry.update();
        } else if (gamepad2.a) {
            robot.armDown();
            telemetry.addData("getLeftArm() position", robot.getLeftArm().getPosition());
            telemetry.addData("getRightArm() position", robot.getRightArm().getPosition());
            telemetry.update();
        } else if (gamepad2.dpad_down) {
            robot.armDown();
            telemetry.addData("getLeftArm() position", robot.getLeftArm().getPosition());
            telemetry.addData("getRightArm() position", robot.getRightArm().getPosition());
            telemetry.update();
        } else {
        }

        // Send telemetry message to signify robot running;

        telemetry.addData("left", "%.2f", robot.getLeftDrive().getPower());
        telemetry.addData("right", "%.2f", robot.getRightDrive().getPower());
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}