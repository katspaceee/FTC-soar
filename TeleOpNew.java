
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.botsquadutil.PID;

import java.util.Arrays;
import java.util.Base64;
import java.util.List;

@TeleOp(name="TeleOp2022NewThisOne", group="2022-2023")
public class TeleOpNew extends OpMode {
    //Time
    private ElapsedTime runTime = new ElapsedTime();
    private double lastTime = 0.0;

    //Motors & Servos
    private DcMotor TR, TL, BL, BR, ARB1, ARB2;
    private Servo CLJ, Claw;

    //Motor Lists
    private List<DcMotor> Chassis;
    private List<DcMotor> ARM;

    private double TRPower = 0.0, TLPower = 0.0, BRPower = 0.0, BLPower = 0.0, clawPos = 0.0;
    private double ARBPosScale = 50.0;
    private double chassisSpeedMultiplier = 0.33;
    private int ARBSpeedMultiplier = 1;

    //Sensors
    private BNO055IMU imu;

    private double proportional = 0.7, robHead = 0.0, lastRx = 0.0;
    private PID angularPID = new PID(0.36, 0.15, 0.05);

//0-
    //Constants
    private double maxChassisRPM = 1200;
    private double wheelRad = 2.283465, robRad = 9.00459;
    private double normalizedARMPow = 1.0;
    private double CLJAngleToNormal = 1.0 / 270.0;

    //This is Spaghetti-code for now, but it gets the job done.
    @Override
    public void init(){

        TR = hardwareMap.get(DcMotor.class, "TR");
        TL = hardwareMap.get(DcMotor.class, "TL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        ARB1 = hardwareMap.get(DcMotor.class, "ARB1");
        ARB2 = hardwareMap.get(DcMotor.class, "ARB2");

        Chassis = Arrays.asList(TR, TL, BR, BL);
        ARM = Arrays.asList(ARB1, ARB2);

        //Initiate Chassis Motors to using encoders
        setRunUsingEncoders(Chassis);
        setZeroPowerBehaviour(Chassis, DcMotor.ZeroPowerBehavior.BRAKE);

        //Initiatite ARM to run to specific position using FTC's provided PID Control Algorithm
        setRunToPosition(ARM, normalizedARMPow);

        //Setting direction for motors
        TR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        ARB1.setDirection(DcMotorSimple.Direction.REVERSE);
        ARB2.setDirection(DcMotorSimple.Direction.FORWARD);

        Claw = hardwareMap.get(Servo.class, "claw");

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop(){
        //Determines the difference in time between frames
        double currentTime = runTime.time();
        double delTime = currentTime - lastTime;
        lastTime = currentTime;

        //Getting data from gamepad1 for movement
        double x = (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)) * 1.1;
        double y = (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
        double rx = (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));

        calculateTargetHead(rx);

        //Calculates angle error of the robot between target head and actual head.
        double angleError = checkStrafeAndHead(x, y, robHead, delTime);


        //Robot-centric strafing
        //---------------------------------------------------------------------//
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) + Math.abs(angleError), 1);
        TRPower = ((y + x) * chassisSpeedMultiplier - angleError) / denominator;
        BLPower = ((y - x) * chassisSpeedMultiplier - angleError) / denominator;
        TLPower = ((y - x) * chassisSpeedMultiplier + angleError) / denominator;
        BRPower = ((y + x) * chassisSpeedMultiplier + angleError) / denominator;
        //---------------------------------------------------------------------//


        //Checks for driver speed shifts
        checkSpeedShifts();

        //Setting motor chassis powers
        TL.setPower(TLPower);
        BL.setPower(BLPower);
        TR.setPower(TRPower);
        BR.setPower(BRPower);

        arm();
        claw();

        //Telemetry for debugging
        telemetry.addData("robHead", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("angleError", angleError);
        telemetry.addData("TargetHead", robHead);
        telemetry.addData("TLPower", TLPower);
        telemetry.addData("TRPower", TRPower);
        telemetry.addData("BLPower", BLPower);
        telemetry.addData("BRPower", BRPower);
        telemetry.addData("ARB1Position", ARB1.getCurrentPosition());
        telemetry.addData("ARB2Position", ARB2.getCurrentPosition());
    }

    //Checks if value given is less than minimum or greater than a max (floors/ceils number if outside a boundry)
    private void checkMinMaxValue(double min, double max, double value){
        if(value < min){
            value = min;
            telemetry.addData("here", true);
        }
        else if(value > max){
            value = max;
        }
    }

    //setsZeroPowerBehaviour on a list of motors given a set behaviour (ZeroPowerBehviour is what the motor will do when given
    //0 power for a frame).
    private void setZeroPowerBehaviour(List<DcMotor> motors, DcMotor.ZeroPowerBehavior behavior){
        for(DcMotor motor : motors){
            motor.setZeroPowerBehavior(behavior);
        }
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

    //sets all motors in a list of motors to mode "Run Using Encoders"
    private void setRunUsingEncoders(List<DcMotor> motors){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //Determines if the robot needs to rotate given error in the expected heading and actual heading.
    public double checkStrafeAndHead(double expectedX, double expectedY, double expectedHeading, double delTime){
        double angleErr = (expectedHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

        return angularPID.output(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, expectedHeading, delTime);
    }

    //Determines if the driver activated in speed shifts
    public void checkSpeedShifts(){
        if(gamepad2.x){
            ARBSpeedMultiplier = 10;
        }
        else if(gamepad2.a){
            ARBSpeedMultiplier = 5;
        }
        else if(gamepad2.b){
            ARBSpeedMultiplier = 3;
        }

        if(gamepad1.x){
            chassisSpeedMultiplier = 0.66;
        }
        else if(gamepad1.a){
            chassisSpeedMultiplier = 0.33;
        }
        else if(gamepad1.b){
            chassisSpeedMultiplier = 0.15;
        }
        else if(gamepad1.y){
            chassisSpeedMultiplier = 0.05;
        }
    }

    //Calculates robot target head in terms of radians
    public void calculateTargetHead(double rx){
        double sign = robHead / Math.abs(robHead);
        robHead -=  calculateDelTargetHead(rx);

        //IMU reads angular displacement in the domain of tangent. Therefore, we must
        //change the orientation of the robot after crossing PI radians.
        //Domain:
        //Clockwise max: PI
        //Counter-Clockwise max: -PI
        if(Math.abs(robHead) > Math.PI){
            robHead = Math.abs(robHead) % Math.PI - sign * Math.PI;
        }
    }

    //Calculates a change in targetHead
    public double calculateDelTargetHead(double delRot){
        return ((wheelRad / robRad) * (delRot * chassisSpeedMultiplier) * 0.6771); //% 6.283185;
    }

    //Stops arm from reaching a max/min height
    public void arm(){
        int ARB1TargetPos = ARB1.getCurrentPosition() + (int)Math.round(gamepad2.left_stick_y * ARBPosScale * ARBSpeedMultiplier);
        int ARB2TargetPos = ARB2.getCurrentPosition() + (int)Math.round(gamepad2.left_stick_y * ARBPosScale * ARBSpeedMultiplier);

        if(ARB1TargetPos > 0 || ARB2TargetPos > 0){
            ARB1TargetPos = 0;
            ARB2TargetPos = 0;
        }

        if(ARB2TargetPos < -6200){
            ARB1TargetPos = -6200;
            ARB2TargetPos = -6200;
        }

        ARB1.setTargetPosition(ARB1TargetPos);
        ARB2.setTargetPosition(ARB2TargetPos);
    }

    //Stops claw from reaching max/min opening
    public void claw(){
        if(gamepad2.right_bumper)
            clawPos += 30;
        else if(gamepad2.left_bumper)
            clawPos -= 30;

        if(clawPos < 0.0){
            clawPos = 0.0;
            telemetry.addData("here", true);
        }
        else if(clawPos > 270.0){
            clawPos = 270.0;
        }

        Claw.setPosition(clawPos * (CLJAngleToNormal));
        telemetry.addData("claw", clawPos);
    }
}
