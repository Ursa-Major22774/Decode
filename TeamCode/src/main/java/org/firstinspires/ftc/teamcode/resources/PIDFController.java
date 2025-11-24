package org.firstinspires.ftc.teamcode.resources;

/**
 * GENERIC PIDF CONTROLLER
 * Used for positional error correction.
 * Logic: "I am here, I want to be there, how hard do I push?"
 */
public class PIDFController {
    public double kP, kI, kD, kF;
    private double lastError = 0;
    private double integralSum = 0;
    private long lastTime = 0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * CALCULATE
     * Call this every loop.
     * @param target Where you want to be
     * @param current Where you actually are
     * @return The power/correction value to apply
     */
    public double calculate(double target, double current) {
        double error = target - current;
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0; // Seconds

        // Integration (Accumulated error)
        integralSum += error * dt;

        // Derivative (Rate of change of error)
        double derivative = (error - lastError) / dt;

        // Store for next loop
        lastError = error;
        lastTime = currentTime;

        // PID Formula
        return (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * target);
    }

    // Reset integral if we settle or change targets drastically
    public void reset() {
        integralSum = 0;
        lastError = 0;
    }
}