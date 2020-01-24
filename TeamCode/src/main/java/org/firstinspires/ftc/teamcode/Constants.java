package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Constants {

    // Servo claws
    public static final double LEFT_MID_SERVO = 0.33;
    public static final double RIGHT_MID_SERVO = 0.52;
    public static final double MAX_CLAW_OFFSET = 0.1;

    // Arm
    public static final double UP_ARM_SPEED = 0.03;
    public static final double DOWN_ARM_SPEED = 0.008;

    // Jewel arm
    public static final double JEWEL_UP_LIMIT = 0;
    public static final double[] JEWEL_STOPS = {0.46, 0.48, 0.5};
    public static final double JEWEL_ARM_SPEED = 0.05;

    // Motors
    public static final String LEFT_DRIVE_NAME = "left_drive";
    public static final String RIGHT_DRIVE_NAME = "right_drive";
    public static final String THIRD_WHEEL_NAME = "third_wheel";

    public static final DcMotor.Direction LEFT_DRIVE_REVERSE = DcMotor.Direction.REVERSE;  // Set to REVERSE if using AndyMark motors
    public static final DcMotor.Direction RIGHT_DRIVE_REVERSE = DcMotor.Direction.FORWARD; // Set to FORWARD if using AndyMark motors
    public static final DcMotor.Direction THIRD_WHEEL_REVERSE = DcMotor.Direction.REVERSE;

    public static final DcMotor.RunMode MOTOR_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    // Servos
    public static final String LEFT_CLAW_NAME = "left_hand";
    public static final String RIGHT_CLAW_NAME = "right_hand";
    public static final String JEWEL_SERVO_NAME = "jewel_arm";
    public static final String LEFT_ARM_NAME = "left_arm";
    public static final String RIGHT_ARM_NAME  = "right_arm";

    public static final Servo.Direction LEFT_ARM_REVERSE = Servo.Direction.REVERSE;
    public static final Servo.Direction RIGHT_ARM_REVERSE = Servo.Direction.FORWARD;

    // Claw variables
    public static final double LEFT_CLAW_MIN   =  0;
    public static final double LEFT_CLAW_MAX   =  0;
    public static final double RIGHT_CLAW_MIN  =  0;
    public static final double RIGHT_CLAW_MAX  =  0;
    
    // Sensors
    public static final String COLOR_SENSOR_NAME = "sensor_color_distance";
    public static final String DISTANCE_SENSOR_NAME = "sensor_color_distance";
    public static final String TOUCH_SENSOR_NAME = "sensor_touch";
}