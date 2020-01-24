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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

public class Drivebase extends Constants {
    
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor thirdWheel;
    public Servo leftArm;
    public Servo rightArm;
    public Servo leftClaw;
    public Servo rightClaw;
    public Servo jewelServo;

    // Sensors
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public TouchSensor sensorTouch;
	
    /* local OpMode members. */
    public HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Drivebase() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "right_drive");
        rightDrive = hwMap.get(DcMotor.class, "left_drive");
        thirdWheel = hwMap.get(DcMotor.class, "third_wheel");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        thirdWheel.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        thirdWheel.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thirdWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftClaw = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        jewelServo = hwMap.get(Servo.class, "jewel_arm");
        leftArm = hwMap.get(Servo.class, "left_arm");
        rightArm = hwMap.get(Servo.class, "right_arm");

        leftClaw.setPosition(LEFT_MID_SERVO - MAX_CLAW_OFFSET);
        rightClaw.setPosition(RIGHT_MID_SERVO + MAX_CLAW_OFFSET);

//        leftClaw.setPosition(LEFT_MID_SERVO);
//        rightClaw.setPosition(RIGHT_MID_SERVO);
        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.FORWARD);
        setArmPosition(.2);

        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");
//        sensorTouch = hwMap.get(TouchSensor.class, "sensor_touch");
    }

    public void setArmPosition(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    public void armUp() {
        leftArm.setPosition(leftArm.getPosition() + UP_ARM_SPEED);
        rightArm.setPosition(rightArm.getPosition() + UP_ARM_SPEED);
    }

    public void armDown() {
        leftArm.setPosition(leftArm.getPosition() - DOWN_ARM_SPEED);
        rightArm.setPosition(rightArm.getPosition() - DOWN_ARM_SPEED);
    }

    public HardwareMap getHardwareMap() { return hwMap; }
    public DcMotor getLeftDrive() { return leftDrive; }
    public DcMotor getRightDrive() { return rightDrive; }
}