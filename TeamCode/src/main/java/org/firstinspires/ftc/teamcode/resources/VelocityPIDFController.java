package org.firstinspires.ftc.teamcode.resources;

/**
 * VELOCITY CONTROLLER
 * specifically for the Flywheel.
 * Now calculates velocity internally based on Encoder Ticks over Time.
 */
public class VelocityPIDFController {
    private double kP, kI, kD, kF;
    private double lastError = 0;
    private double integralSum = 0;

    // State variables for velocity calculation
    private double lastPosition = 0;
    private long lastTime = 0;

    public VelocityPIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * CALCULATE POWER
     * @param targetVelocity The target velocity in TICKS PER SECOND
     * @param currentPosition The current raw encoder value
     * @return Motor power (0.0 to 1.0)
     */
    public double calculate(double targetVelocity, double currentPosition) {
        long currentTime = System.nanoTime();

        // Handle first run to prevent massive derivative/velocity spikes
        if (lastTime == 0) {
            lastTime = currentTime;
            lastPosition = currentPosition;
            return kF * targetVelocity; // Feedforward only on first frame
        }

        // Calculate Time Delta (Seconds)
        double dt = (currentTime - lastTime) / 1E9;

        // Prevent divide by zero if loop is instantaneous (unlikely but safe)
        if (dt < 1E-9) return 0;

        // 1. CALCULATE VELOCITY (Ticks per Second)
        double currentVelocity = (currentPosition - lastPosition) / dt;

        // 2. CALCULATE ERROR
        double error = targetVelocity - currentVelocity;

        // 3. INTEGRAL (Sum of error over time)
        integralSum += error * dt;

        // 4. DERIVATIVE (Rate of change of error)
        double derivative = (error - lastError) / dt;

        // Update state for next loop
        lastError = error;
        lastTime = currentTime;
        lastPosition = currentPosition;

        // PIDF Formula
        return (kF * targetVelocity) + (kP * error) + (kI * integralSum) + (kD * derivative);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        lastPosition = 0;
    }
}