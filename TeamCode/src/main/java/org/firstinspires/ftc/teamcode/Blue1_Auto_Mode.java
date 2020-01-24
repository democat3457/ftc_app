package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "We are BLUE Team", group = "Autonomous")
//@Disabled
public class Blue1_Auto_Mode extends LinearOpMode {
    public static final String TAG = "Vuforia VuMark Sample";
    Drivebase robot = new Drivebase();
    OpenGLMatrix lastLocation = null;
    /**
     * {@link} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();

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

    public void turn(double leftPower, double rightPower, double time) {
        double startTime = runtime.time(TimeUnit.SECONDS);
        while (runtime.time(TimeUnit.SECONDS) < startTime + time) {
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        int count = 0;

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
        robot.jewelServo.setPosition(robot.JEWEL_UP_LIMIT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AUzwlmf/////AAAAGeF4fSfKj0E/sO0hGm4OxLlklqv9XKWdCvCrqpCCorEeqIADLYyvs7tJkvKgL8R7HQ01gUBiA4oF/kqDjRADZOVXUqPN8RLQtERPIkWxE9T1AKHOw5WuV3N8T5j2eMbjyoZbklh5jDILSWiDS3gZRKV5zHTxxi3508j9GfvzAexl+4kOznoXN8fWfiPcTMUmPGp4yERKhbAmksRlTggK83mFxfyc4yB1zNptmSvJ9JP3FqhYxfjtnelYPDQwmQMjDd+P6GNQgZpt79wlYqlvOFmOdcs85WTbrItBpxc6yHLxKJTHclzuTKYpSxErgEOsfQ8vWJrSHmknv+N6wv161rFw5lrz2j2X4/mrS37X00Aj";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();
        runtime.reset();
        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
//                telemetry.addData("VuMark", "%s visible", vuMark);


                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
//                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
//                telemetry.addData("VuMark", "not visible");
            }

            if (runtime.time(TimeUnit.SECONDS) > 30) {
                break;
            }

//            telemetry.update();

            // Perform the Autonomous steps for the Jewel Arm detection.
            if (count < 1) {
                int move = 0;
                int max = 0;
                char[] colors = new char[3];
                while (runtime.time(TimeUnit.SECONDS) < 30 && move == 0 && max != 3) {
                    for (int i = 0; i < 3; i++) {
                        robot.jewelServo.setPosition(robot.JEWEL_STOPS[i]);

                        sleep(1000);
//                    double alpha = robot.sensorColor.alpha();
                        double red = robot.sensorColor.red();
                        double blue = robot.sensorColor.blue();
//                    double green = robot.sensorColor.green();
                        if (red > blue) {
                            colors[i] = 'R';
                        } else if (red < blue) {
                            colors[i] = 'B';
                        }
                    }
                    sleep(200);

                    //Three out of three right
                    if (colors[0] == 'R' && colors[1] == 'R' && colors[2] == 'R') {
                        move = -1;
                    } else if (colors[0] == 'B' && colors[1] == 'B' && colors[2] == 'B') {
                        move = 1;
                    }
                    max += 1;
                    //Two out of three right
                /*
                if ((colors[0]=='R'&&colors[1]=='R')||(colors[1]=='R'&&colors[2]=='R')||(colors[0]=='R'&&colors[2]=='R')){
                    move = -1;
                } else if ((colors[0]=='B'&&colors[1]=='B')||(colors[1]=='B'&&colors[2]=='B')||(colors[0]=='B'&&colors[2]=='B')){
                    move = 1;
                }
                */
                }

                if (move == -1) {
                    forward(-0.45, 0.5);

                } else if (move == 1) {
                    forward(0.45, 0.5);
                }
                sleep(2000);
                robot.jewelServo.setPosition(robot.JEWEL_UP_LIMIT);
                telemetry.addData("Detected colors", String.valueOf(colors));
                telemetry.addData("Color(outright", colors[0] + colors[1] + colors[2]);
                telemetry.update();

                sleep(2000);
//
//                forward(-2, 0.5);
//                turn(0, 1, 0.5);
//                forward(-1, 0.5);
//
//                robot.leftClaw.setPosition(robot.LEFT_MID_SERVO + robot.MAX_CLAW_OFFSET);
//                robot.rightClaw.setPosition(robot.RIGHT_MID_SERVO - robot.MAX_CLAW_OFFSET);

                sleep(5000);
                count += 1;
            }
        }
    }
}
