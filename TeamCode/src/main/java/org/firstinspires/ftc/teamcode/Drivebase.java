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

import java.lang.invoke.ConstantCallSite;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Class that defines drivebase components and sensors.
 */

public class Drivebase extends Constants {
    
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor thirdWheel;
    private Servo leftArm;
    private Servo rightArm;
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo jewelServo;

    // Sensors
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    private TouchSensor sensorTouch;
	
    /* local OpMode members. */
    private HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Drivebase() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, Constants.LEFT_DRIVE_NAME);
        rightDrive = hwMap.get(DcMotor.class, Constants.RIGHT_DRIVE_NAME);
        thirdWheel = hwMap.get(DcMotor.class, Constants.THIRD_WHEEL_NAME);
        leftDrive.setDirection(Constants.LEFT_DRIVE_REVERSE);
        rightDrive.setDirection(Constants.RIGHT_DRIVE_REVERSE);
        thirdWheel.setDirection(Constants.THIRD_WHEEL_REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        thirdWheel.setPower(0);

        leftDrive.setMode(Constants.MOTOR_RUN_MODE);
        rightDrive.setMode(Constants.MOTOR_RUN_MODE);
        thirdWheel.setMode(Constants.MOTOR_RUN_MODE);

        // Define and initialize ALL installed servos.
        leftClaw = hwMap.get(Servo.class, Constants.LEFT_CLAW_NAME);
        rightClaw = hwMap.get(Servo.class, Constants.RIGHT_CLAW_NAME);
        jewelServo = hwMap.get(Servo.class, Constants.JEWEL_SERVO_NAME);
        leftArm = hwMap.get(Servo.class, Constants.LEFT_ARM_NAME);
        rightArm = hwMap.get(Servo.class, Constants.RIGHT_ARM_NAME);

        leftClaw.setPosition(Constants.LEFT_MID_SERVO - Constants.MAX_CLAW_OFFSET);
        rightClaw.setPosition(Constants.RIGHT_MID_SERVO + Constants.MAX_CLAW_OFFSET);

        // leftClaw.setPosition(LEFT_MID_SERVO);
        // rightClaw.setPosition(RIGHT_MID_SERVO);
        leftArm.setDirection(Constants.LEFT_ARM_REVERSE);
        rightArm.setDirection(Constants.RIGHT_ARM_REVERSE);
        setArmPosition(.2);

        sensorColor = hwMap.get(ColorSensor.class, Constants.COLOR_SENSOR_NAME);
        sensorDistance = hwMap.get(DistanceSensor.class, Constants.DISTANCE_SENSOR_NAME);
        sensorTouch = hwMap.get(TouchSensor.class, Constants.TOUCH_SENSOR_NAME);
    }

    public void setArmPosition(double position) {
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }

    public void armUp() {
        leftArm.setPosition(leftArm.getPosition() + Constants.UP_ARM_SPEED);
        rightArm.setPosition(rightArm.getPosition() + Constants.UP_ARM_SPEED);
    }

    public void armDown() {
        leftArm.setPosition(leftArm.getPosition() - Constants.DOWN_ARM_SPEED);
        rightArm.setPosition(rightArm.getPosition() - Constants.DOWN_ARM_SPEED);
    }

    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thirdWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(Constants.MOTOR_RUN_MODE);
        rightDrive.setMode(Constants.MOTOR_RUN_MODE);
        thirdWheel.setMode(Constants.MOTOR_RUN_MODE);
    }

    public void resetEncoders(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Constants.MOTOR_RUN_MODE);
    }

    public void resetEncoders(DcMotor motor, DcMotor.RunMode runMode) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
    }

    public HardwareMap getHardwareMap() { return hwMap; }

    /** 
     * Gets left drive 
     * @return DcMotor
     */
    public DcMotor getLeftDrive() { return leftDrive; }
    public DcMotor getRightDrive() { return rightDrive; }
    public DcMotor getThirdWheel() { return thirdWheel; }

    public Servo getLeftClaw() { return leftClaw; }
    public Servo getRightClaw() { return rightClaw; }

    public Servo getJewelArm() { return jewelServo; }

    public Servo getLeftArm() { return leftArm; }
    public Servo getRightArm() { return rightArm; }

    public ColorSensor getColorSensor() { return sensorColor; }
    public DistanceSensor getDistanceSensor() { return sensorDistance; }
    public TouchSensor getTouchSensor() { return sensorTouch; }
}