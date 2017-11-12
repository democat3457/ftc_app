package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RotateServo", group = "Linear Opmode")
//@Disabled
public class RotateServo extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    Servo servo;
    double servoPosition=0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status","Initialized");
        telemetry.update();
        servo=hardwareMap.servo.get("servo");
        servo.setPosition(servoPosition);
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            telemetry.addData("Status","Run Time: ",runtime.toString());
            telemetry.update();
            idle();
        }

        servoPosition=0.5;
        servo.setPosition(servoPosition);
        sleep(2000);

        servoPosition=1.0;
        servo.setPosition(servoPosition);
    }
}
