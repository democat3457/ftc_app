package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red1 Auto Mode", group = "Linear Opmode")
//@Disabled
public class Red1_Auto_Mode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    HardwarePushbot robot = new HardwarePushbot();




    /*
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 4.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    */



    /*
    public void driveLine (double speed, double inches){
        int startPos = robot.leftArm.getCurrentPosition();
        robot.leftArm.setPower(0.1);
        while (Math.abs(robot.leftArm.getCurrentPosition()-startPos)<=(int)(inches*(1120/4))){

            telemetry.update();

        }
        robot.leftArm.setPower(0);

        sleep(200);



    }
    */


    public void forward(double power, double time) {
        double startTime = runtime.time(TimeUnit.SECONDS);
        while (runtime.time(TimeUnit.SECONDS) < startTime + time) {
            robot.leftDrive.setPower(power);
            robot.rightDrive.setPower(power);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /*
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
        */
        robot.init(hardwareMap);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        robot.jewelServo.setPosition(robot.JEWEL_UP_LIMIT);
        double jewelArmPosition = robot.JEWEL_UP_LIMIT;

        waitForStart();
        runtime.reset();

//        while(opModeIsActive()) {
        telemetry.addData("Status", "About to set arm to down pos");
        telemetry.update();
        // move jewel arm to down position
        while (jewelArmPosition >= robot.JEWEL_DOWN_LIMIT) {
            jewelArmPosition = jewelArmPosition - robot.JEWEL_ARM_SPEED;
            robot.jewelServo.setPosition(jewelArmPosition);
            telemetry.addData("Jewel Arm down", String.format(Locale.US, "%.02f", jewelArmPosition));
            telemetry.update();
            sleep(50);
        }

        sleep(1000);
        telemetry.addData("Status", "About to move jewel");
        telemetry.update();
        String color = "";
        if (robot.sensorColor.red() > robot.sensorColor.blue()) {
            // moving forward to knock off the jewel
            telemetry.addData("Status", "About to move forward");
            telemetry.update();
            forward(0.35, 0.10);
            color = "red";
        } else if (robot.sensorColor.red() < robot.sensorColor.blue()) {
            // moving backwards to knock off the jewel
            telemetry.addData("Status", "About to move backward");
            telemetry.update();
            forward(-0.35, 0.10);
            color = "blue";
        }
        telemetry.addData("Status", "About to set arm to up pos");
        // move jewel arm to up position
        telemetry.addData("Jewel Arm down", String.format(Locale.US, "%.02f", jewelArmPosition));
        telemetry.update();
        while (jewelArmPosition <= robot.JEWEL_UP_LIMIT) {
            jewelArmPosition = jewelArmPosition + robot.JEWEL_ARM_SPEED;
            robot.jewelServo.setPosition(jewelArmPosition);
            telemetry.addData("Jewel Arm down", String.format(Locale.US, "%.02f", jewelArmPosition));
            telemetry.update();
            sleep(50);
        }
        sleep(1000);
        if (color.equals("red")) {
            forward(-0.45, 0.1);

        } else if (color.equals("blue")) {
            forward(0.45, 0.1);
        }

        // prints current run time to the screen
        telemetry.addData("Status", "Run Time: ", runtime.toString());
        sleep(2000);

//        telemetry.addData("Distance (cm)",
//                String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));


        telemetry.addData("Alpha", robot.sensorColor.alpha());
        telemetry.addData("Red  ", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue ", robot.sensorColor.blue());

        telemetry.update();

        sleep(7000);
//        }
//        robot.jewelServo.setPosition(robot.JEWEL_UP_LIMIT);


    }
}
