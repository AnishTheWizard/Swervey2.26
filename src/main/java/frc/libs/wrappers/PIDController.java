package frc.libs.wrappers;

public class PIDController {
    double kP, kI, kD;

    double setpoint;

    double currentTime, prevTime;

    double lastErr;


    public PIDController(double kp, double ki, double kd) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;

        prevTime = System.currentTimeMillis();
        lastErr = 0;
    }

    public PIDController(double[] gains) {
        kP = gains[0];
        kI = gains[1];
        kD = gains[2];

        prevTime = System.currentTimeMillis();
        lastErr = 0;
    }

    public PIDController() {
        kP = 0;
        kI = 0;
        kD = 0;

        prevTime = System.currentTimeMillis();
        lastErr = 0;
    }

    public double calculate(double current, double setpoint) {
        double elapsed = System.currentTimeMillis() - prevTime;
        double err = setpoint - current;

        double integratedErr = err * elapsed;

        double derivedErr = (err - lastErr)/elapsed;

        double output = kP * err + kI * integratedErr + kD * derivedErr;

        lastErr = err;
        prevTime = System.currentTimeMillis();

        return output;
    }

    public double calculate(double setpoint) {
        return calculate(0, setpoint);
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public void setPID(double[] gains) {
        this.kP = gains[0];
        this.kI = gains[1];
        this.kD = gains[2];
    }
}
