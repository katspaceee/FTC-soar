package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.botsquadutil.PID;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="Auton", group="2022-2023")
public class auto extends LinearOpMode {

    BNO055IMU imu;
    DcMotor TL, TR, BL, BR, ARB1, ARB2;
    List<DcMotor> driveMotors;
    List<DcMotor> ARM;
    Servo claw;
    double proportional = 1.0;
    double robPosX = 0.0, robPosY = 0.0;
    double lastTime = 0.0;

    double ARBWheelRad = 0.0;
    double ARBWheelCirc = 2.0 * Math.PI * ARBWheelRad;
    int ARBTickPerRev = 28;

    ElapsedTime runTime = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    private PID angularPID = new PID(0.36, 0.15, 0.05);
    private PID linearPID = new PID(0.2, 0.0, 0.0);

    //New Configs for C270
    //Units are in pixels
    double fx = 1996.17;
    double fy = 1996.17;
    double cx = 319.5;
    double cy = 239.5;
    double TRPower, TLPower, BLPower, BRPower;

    // UNITS ARE METERS
    double tagsize = 0.508;

    //Tag IDs of Sleeve
    int id_Left = 5;
    int id_Mid = 13;
    int id_Right = 18;

    AprilTagDetection tagOfInterest = null;

    Pose2d startPose = new Pose2d(35.25, -63.4725, 0);
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() {
        //Initialize Hardware
        initHardware();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        //Wait for start and detect the sleeve
        detectWhileWaitForStart();

        /*
        Trajectory path1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(23.5)
                .build();
        Trajectory path2 = drive.trajectoryBuilder(path1.end())
                .splineTo(new Vector2d(5.02,-28.52), Math.toRadians(45))
                .build();
        */

        if(tagOfInterest.id == id_Left){
            /*
            drive.followTrajectory(path1);
            drive.followTrajectory(path2);
            */

            goTo(-0.596, 0.0, 0.0, 0.15, 0.00899);
        }

        else if(tagOfInterest.id == id_Mid){
            /*Trajectory endPath = drive.trajectoryBuilder(path2.end(), true)
                    .splineTo(new Vector2d(35.25, -35.25), Math.toRadians(90))
                    .build();
            drive.followTrajectory(path1);
            drive.followTrajectory(path2);
            drive.followTrajectory(endPath);
            */
        }

        else if(tagOfInterest.id == id_Right){
            /*
            Trajectory endPath = drive.trajectoryBuilder(path2.end(), true)
                    .splineTo(new Vector2d(58.75, -35.25), Math.toRadians(90))
                    .build();
            drive.followTrajectory(path1);
            drive.followTrajectory(path2);
            drive.followTrajectory(endPath);
            */


        }
        //default path
        else {
            if(gamepad1.right_bumper && gamepad1.left_bumper){
                goTo(0.0, 0.584, 0.0, 0.15, 0.00899);
            }
        }
    }

    //Units: inches
    private void armGoTo(double height){
        int ticks = (int)Math.round(height / ARBWheelCirc) * ARBTickPerRev;

        ARB1.setTargetPosition(ticks);
        ARB2.setTargetPosition(ticks);
    }

    private void initHardware(){
        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        driveMotors = Arrays.asList(TL, TR, BL, BR);
        ARM = Arrays.asList(ARB1, ARB2);

        ARB1 = hardwareMap.get(DcMotor.class, "ARB1");
        ARB2 = hardwareMap.get(DcMotor.class, "ARB2");

        claw = hardwareMap.get(Servo.class, "claw");

        TR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        TL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        ARB1.setDirection(DcMotorSimple.Direction.REVERSE);
        ARB2.setDirection(DcMotorSimple.Direction.FORWARD);

        setRunToPosition(ARM, 0.25);
        setMode(driveMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehaviour(driveMotors, DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Initializing AprilTagDetection (Detection of the sleeve)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //Originally 800 and 448
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    //Detects the sleeve while waiting for the start
    private void detectWhileWaitForStart(){
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == id_Left || tag.id == id_Mid || tag.id == id_Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
    }

    public void setMode(List<DcMotor> motorList, DcMotor.RunMode runmode){
        for(DcMotor motor : motorList){
            motor.setMode(runmode);
        }
    }

    public void setZeroPowerBehaviour(List<DcMotor> motorList, DcMotor.ZeroPowerBehavior behaviour){
        for(DcMotor motor : motorList){
            motor.setZeroPowerBehavior(behaviour);
        }
    }

    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    //Robot-centric
    //Use meters
    public void goTo(double x, double y, double heading, double motorScalingFactor, double minErr){
        double errX = x - imu.getPosition().x;
        double errY = y - imu.getPosition().y;

        while(Math.abs(errX) > minErr && Math.abs(errY) > minErr) {
            double deltaTime = getDelTime();

            PID.tunedOut tndOut = checkStrafeAndHead(x, y, heading, deltaTime);

            double tunedAngle = tndOut.angle;
            double tunedX = tndOut.x;
            double tunedY = tndOut.y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(tunedAngle) + Math.abs(tunedY) + Math.abs(tunedX), 1);
            double leftPow = ((y + tunedY) - (x + tunedX)) * motorScalingFactor;
            double rightPow = ((y + tunedY) + (x + tunedX)) * motorScalingFactor;

            TRPower = (rightPow - tunedAngle) / denominator;
            BLPower = (leftPow - tunedAngle) / denominator;
            TLPower = (leftPow + tunedAngle) / denominator;
            BRPower = (rightPow + tunedAngle) / denominator;

            TR.setPower(TRPower);
            BL.setPower(BLPower);
            TL.setPower(TLPower);
            BR.setPower(BRPower);

            errX = x - imu.getPosition().x;
            errY = y - imu.getPosition().y;
        }
    }

    public void rotTo(double angle, double motorScalingValue, double minErr){
        double angleErr = (angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

        while(Math.abs(angleErr) > minErr){
            double deltaTime = getDelTime();

            PID.tunedOut tndOut = checkStrafeAndHead(0.0, 0.0, angle, deltaTime);

            double tunedAngle = tndOut.angle;

            TR.setPower(-tunedAngle);
            BL.setPower(-tunedAngle);
            TL.setPower(tunedAngle);
            TR.setPower(tunedAngle);

            angleErr = (angle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        }
    }

    public double getDelTime(){
        double currentTime = runTime.time();
        lastTime = currentTime;

        return currentTime - lastTime;
    }

    public PID.tunedOut checkStrafeAndHead(double expectedX, double expectedY, double expectedHeading, double delTime){
        double tunedX = linearPID.output(imu.getPosition().x, expectedX, delTime);
        double tunedY = linearPID.output(imu.getPosition().y, expectedY, delTime);
        double tunedAngle = angularPID.output(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, expectedHeading, delTime);

        PID.tunedOut output = new PID.tunedOut(tunedX, tunedY, tunedAngle);

        return output;
    }

    //sets all motors in a list of motors to mode "Run To Position"
    private void setRunToPosition(List<DcMotor> motors, double normalizedPow){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(normalizedPow);
        }
    }
}
