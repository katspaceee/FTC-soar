
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

    private double proportional = 0.6, robHead = 0.0, lastRx = 0.0;
    private PID angularPID = new PID(0.7, 0.0, 0.0);

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
        TL.setDirection(DcMotorSimple.Direction.FORWARD);
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
        //Getting data from gamepad1 for movement
        double currentTime = runTime.time();
        double delTime = currentTime - lastTime;
        lastTime = currentTime;

        double x = (gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x)) * 1.1;
        double y = (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
        double rx = (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x));

        double angleError = checkStrafeAndHead(x, y, robHead, delTime);

        double delRot = rx - lastRx;
        lastRx = rx;
        robHead += calculateDelTargetHead(delRot);

        telemetry.addData("delRot", delRot);
        telemetry.addData("robHead", robHead);
        telemetry.addData("angleError", angleError);
        telemetry.addData("TargetHead", robHead);

        //Field-centric strafing
        //---------------------------------------------------------------------//
        /*double head = -imu.getAngularOrientation().firstAngle;

        //Apply rotation matrix upon vector [x, y] to get the subsequent x and y powers.
        double rotX = x * Math.cos(head) - y * Math.sin(head);
        double rotY = x * Math.sin(head) + y * Math.cos(head);

        double scale = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double TRPower = (rotY + rotX + rx) / scale;
        double BLPower = (rotY - rotX + rx) / scale;
        double TLPower = (rotY - rotX - rx) / scale;
        double BRPower = (rotY + rotX - rx) / scale;*/
        //---------------------------------------------------------------------//


        //Robot-centric strafing
        //---------------------------------------------------------------------//
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) + Math.abs(angleError), 1);
        TRPower = ((y + x + rx) * chassisSpeedMultiplier - angleError) / denominator;
        BLPower = ((y - x + rx) * chassisSpeedMultiplier - angleError) / denominator;
        TLPower = ((y - x - rx) * chassisSpeedMultiplier + angleError) / denominator;
        BRPower = ((y + x - rx) * chassisSpeedMultiplier + angleError) / denominator;
        //---------------------------------------------------------------------//

        checkSpeedShifts();

        TL.setPower(TLPower);
        BL.setPower(BLPower);
        TR.setPower(TRPower);
        BR.setPower(BRPower);

        arm();
        claw();

        telemetry.addData("TLPower", TLPower);
        telemetry.addData("TRPower", TRPower);
        telemetry.addData("BLPower", BLPower);
        telemetry.addData("BRPower", BRPower);
        telemetry.addData("ARB1Position", ARB1.getCurrentPosition());
        telemetry.addData("ARB2Position", ARB2.getCurrentPosition());
    }

    private void checkMinMaxValue(double min, double max, double value){
        if(value < min){
            value = min;
            telemetry.addData("here", true);
        }
        else if(value > max){
            value = max;
        }
    }


    private void setZeroPowerBehaviour(List<DcMotor> motors, DcMotor.ZeroPowerBehavior behavior){
        for(DcMotor motor : motors){
            motor.setZeroPowerBehavior(behavior);
        }
    }

    private void setRunToPosition(List<DcMotor> motors, double normalizedPow){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(normalizedPow);
        }
    }

    private void setRunUsingEncoders(List<DcMotor> motors){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double checkStrafeAndHead(double expectedX, double expectedY, double expectedHeading, double delTime){
        double angleErr = (expectedHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

        return angularPID.output(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle, expectedHeading, delTime) / 3.0;
    }

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

    public double calculateDelTargetHead(double delRot){
        return ((wheelRad / robRad) * (delRot * chassisSpeedMultiplier) * 0.6771) % 6.283185;
    }

    public void arm(){
        int ARB1TargetPos = ARB1.getCurrentPosition() + (int)Math.round(gamepad2.left_stick_y * ARBPosScale * ARBSpeedMultiplier);
        int ARB2TargetPos = ARB2.getCurrentPosition() + (int)Math.round(gamepad2.left_stick_y * ARBPosScale * ARBSpeedMultiplier);

        if(ARB1TargetPos > 0 || ARB2TargetPos > 0){
            ARB1TargetPos = 0;
            ARB2TargetPos = 0;
        }

        if(ARB2TargetPos < -6150){
            ARB1TargetPos = -6150;
            ARB2TargetPos = -6150;
        }

        ARB1.setTargetPosition(ARB1TargetPos);
        ARB2.setTargetPosition(ARB2TargetPos);
    }

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
