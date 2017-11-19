package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Main_Autonomous_Mode", group = "Linear Opmode")
//@Disabled
public class Main_Autonomous_Mode extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePushbot robot = new HardwarePushbot();
    Servo jewelServo;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 4.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    final double JEWEL_UP_LIMIT   = 0.4;
    final double JEWEL_DOWN_LIMIT = 0.85 ;
    final double JEWEL_ARM_SPEED  = 0.1 ;


    public void driveLine (double speed, double inches){
        int startPos = robot.leftArm.getCurrentPosition();
        robot.leftArm.setPower(0.1);
        while (Math.abs(robot.leftArm.getCurrentPosition()-startPos)<=(int)(inches*(1120/4))){

            telemetry.update();

        }
        robot.leftArm.setPower(0);

        sleep(200);



    }

    @Override
    public void runOpMode() throws InterruptedException {
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        telemetry.addData("Status","Initialized");
        telemetry.update();
        jewelServo=hardwareMap.servo.get("jewel_servo");
        jewelServo.setPosition(JEWEL_UP_LIMIT);
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            telemetry.addData("Status","Run Time: ",runtime.toString());
            telemetry.update();
            while(jewelServo.getPosition()<JEWEL_DOWN_LIMIT) {
                jewelServo.setPosition(jewelServo.getPosition() + JEWEL_ARM_SPEED);
            }
            // send the info back to driver station using telemetry function.
            telemetry.addData("Jewel Arm", "Position: ", jewelServo.getPosition());
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());

            telemetry.update();
        }
        jewelServo.setPosition(JEWEL_UP_LIMIT);
    }
}
