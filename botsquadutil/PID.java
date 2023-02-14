package org.firstinspires.ftc.teamcode.botsquadutil;

public class PID {
    private double errorLast = 0.0;
    private double timeLast = 0.0;
    public PIDCoefficients coefficients;

    //PID Control Constants
    public class PIDCoefficients{
        public double kp, ki, kd;

        PIDCoefficients(double p, double i, double d){
            this.kp = p;
            this.ki = i;
            this.kd = d;
        }
    }

    public static class tunedOut{
        public double x, y, angle;

        public tunedOut(double x, double y, double angle){
            this.x = x;
            this.y = y;
            this.angle = angle;
        }
    }

    //If you don't supply PIDCoefficients, the default coefficients are set to 0, resulting in no correction
    public PID(){
        coefficients = new PIDCoefficients(0.0, 0.0, 0.0);
    }
    public PID(double p, double i, double d) { this.coefficients = new PIDCoefficients(p, i, d); }

    public PID(PIDCoefficients coefficients){
        this.coefficients = coefficients;
    }

    public double output(double empiricalOut, double desiredOut, double deltaTime){
        double error = calculateError(empiricalOut, desiredOut);

        double deltaError = error - errorLast;
        errorLast = error;

        double output = calculateProportional(error) + calculateIntegral(deltaError, deltaTime) + calculateDerivative(deltaError, deltaTime);


        return output;
    }

    protected double calculateProportional(double error){
        return coefficients.kp * error;
    }

    protected double calculateIntegral(double deltaError, double deltaTime){
        return (coefficients.ki * deltaError * deltaTime);
    }

    protected double calculateDerivative(double deltaError, double deltaTime){
        return (coefficients.kd * (deltaError / deltaTime));
    }

    private double calculateError(double empiricalOut, double desiredOut){
        return (desiredOut - empiricalOut);
    }
}
