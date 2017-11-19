package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RotateServo", group = "Linear Opmode")
//@Disabled
public class JewelServo extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    HardwarePushbot robot = new HardwarePushbot();
    Servo jewelServo;
    double servoPosition=0.0;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 4.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


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
        telemetry.addData("Status","Initialized");
        telemetry.update();
        jewelServo=hardwareMap.servo.get("jewel_servo");
        jewelServo.setPosition(servoPosition);
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            telemetry.addData("Status","Run Time: ",runtime.toString());
            telemetry.update();
            idle();
        }

        servoPosition=0.5;
        jewelServo.setPosition(servoPosition);
        sleep(2000);

        servoPosition=1.0;
        jewelServo.setPosition(servoPosition);
    }
}
