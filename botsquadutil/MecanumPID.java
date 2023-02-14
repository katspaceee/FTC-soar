package org.firstinspires.ftc.teamcode.botsquadutil;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.botsquadutil.Math.Vec2D;

public class MecanumPID extends PID {
    public static class Constants{
        public static final double TICKS_PER_REV_TL = 28;
        public static final double TICKS_PER_REV_TR = 28;
        public static final double TICKS_PER_REV_BL = 28;
        public static final double TICKS_PER_REV_BR = 28;

        public static final double MAX_RPM = 150;

        public static final double MAX_VELOCITY = rpmToVelocity(MAX_RPM);

        //public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

        public static double WHEEL_RADIUS = 1.88976; // in
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        public static double TRACK_WIDTH = 1; // in


        public static double kA = 0;
        public static double kStatic = 0;

        public static double MAX_VEL = 30;
        public static double MAX_ACCEL = 30;
        public static double MAX_ANG_VEL = Math.toRadians(60);
        public static double MAX_ANG_ACCEL = Math.toRadians(60);


        public static double encoderTicksToInches(double ticks, double TICKS_PER_REV) {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        }

        public static double rpmToVelocity(double rpm) {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        }

        public static double getMotorVelocityF(double ticksPerSecond) {
            // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
            return 32767 / ticksPerSecond;
        }
    }

    public class MotorPower {
        public double TLPower;
        public double TRPower;
        public double BLPower;
        public double BRPower;

        public MotorPower(double TLPower, double TRPower, double BLPower, double BRPower){
            this.TLPower = TLPower;
            this.TRPower = TRPower;
            this.BLPower = BLPower;
            this.BRPower = BRPower;
        }

        public MotorPower(){
            TLPower = TRPower = BLPower = BRPower = 0.0;
        }
    }

    private double x, y, rx;
    public MotorPower motorPower;

    public MecanumPID(){
        x = y = rx = motorPower.TLPower = motorPower.TRPower = motorPower.BLPower = motorPower.BRPower = 0.0;
    }

    public MecanumPID(MotorPower motorPower){
        this.motorPower = motorPower;
    }

    public void runCheck(Vec2D currentStrafeVelocity, Vec2D desiredStrafeVelocity, double currentHeading, double desiredHeading, double time){
        checkHeading(currentHeading, desiredHeading, time);
        checkDir(currentStrafeVelocity, desiredStrafeVelocity, time);

        double denominator = Math.abs(y) + Math.abs(x) + Math.abs(rx);

        motorPower.TRPower +=  (y + x + rx) / denominator;
        motorPower.BRPower += (y - x + rx) / denominator;
        motorPower.BLPower += (y - x - rx) / denominator;
        motorPower.TLPower += (y + x - rx) / denominator;

    }

    //Angle (radians)
    private void checkHeading(double currentHeading, double desiredHeading, double time){
        rx = output(currentHeading, desiredHeading, time);
    }

    //Velocity (inches/sec)
    private void checkDir(Vec2D currentStrafeVelocity, Vec2D desiredStrafeVelocity, double time){
        x = output(currentStrafeVelocity.x, desiredStrafeVelocity.x, time) / Constants.MAX_VELOCITY;
        y = output(currentStrafeVelocity.y, desiredStrafeVelocity.y, time) / Constants.MAX_VELOCITY;
    }
}
