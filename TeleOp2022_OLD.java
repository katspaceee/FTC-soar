package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Base64;
import java.util.List;

@TeleOp(name="TeleOp2022New", group="2022-2023")
public class TeleOpNew extends OpMode {
    private DcMotor TR, TL, BL, BR, ARB1, ARB2;
    private DcMotor[] MotorList = {TR, TL, BL, BR, ARB1, ARB2};
    private DcMotor[] ArmMotorList = {ARB1, ARB2};
    private double ARJAngleToTick, ARBAngleToTick, ARJAngle, ARBAngle;
    private Servo CLJ, Claw;
    private double clawPos = 0.0;
    private ElapsedTime time = new ElapsedTime();
    private AnalogSensor potentiometer;
    private double potVoltagetoAngle, potOffset, CLJOffset, CLJAngleToNormal;
    private BNO055IMU imu;

    @Override
    public void init(){
        TR = hardwareMap.get(DcMotor.class, "TR");
        TL = hardwareMap.get(DcMotor.class, "TL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        ARB1 = hardwareMap.get(DcMotor.class, "ARB1");
        ARB2 = hardwareMap.get(DcMotor.class, "ARB2");

        TR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ARB1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARB1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ARB2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARB2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        ARB2.setDirection(DcMotorSimple.Direction.REVERSE);
        
        Claw = hardwareMap.get(Servo.class, "claw");

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        ARBAngle = 0.0;
        ARJAngle = 0.0;

        CLJAngleToNormal = 1.0 / 270.0;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop(){
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double head = -imu.getAngularOrientation().firstAngle;

        //Apply rotation matrix upon vector [x, y] to get the subsequent x and y powers.
        double rotX = x * Math.cos(head) - y * Math.sin(head);
        double rotY = x * Math.sin(head) + y * Math.cos(head);

        double scale = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        TL.setPower((rotY + rotX + rx) / scale);
        BL.setPower((rotY - rotX + rx) / scale);
        TR.setPower((rotY - rotX - rx) / scale);
        BR.setPower((rotY + rotX - rx) / scale);

        ARBAngle += 3.0 * gamepad2.left_stick_y;
        ARB1.setTargetPosition((int)(ARBAngle * (ARBAngleToTick)));
        ARB2.setTargetPosition((int)(ARBAngle * (ARBAngleToTick)));


        //double clawAngle = potVoltagetoAngle * potentiometer.readRawVoltage() + potOffset;
        double a = 0.0;
        double b = 0.0;
        if(gamepad2.a)
            a = 1.0;
        if(gamepad2.b)
            b = -1.0;



        if(gamepad2.right_bumper)
            Claw.setPosition(clawPos - (1.0 * CLJAngleToNormal));

        if(gamepad2.left_bumper)
            Claw.setPosition(clawPos + (1.0 * CLJAngleToNormal));

        double CLJAngle = ARJAngle + CLJOffset + a + b;
        CLJ.setPosition(CLJAngle * CLJAngleToNormal);
    }


    private void initMotorEncoders(DcMotor[] motors){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void resetMotorEncoders(DcMotor[] motors){
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
