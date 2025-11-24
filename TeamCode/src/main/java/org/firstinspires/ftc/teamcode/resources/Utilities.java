package org.firstinspires.ftc.teamcode.resources;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit; // Import added here
import java.util.List;

/**
 * UTILITIES CLASS
 * Static methods to help with hardware optimization and math.
 */
public class Utilities {

    /**
     * SET BULK READ AUTO
     * CRITICAL: Run this at the start of every OpMode.
     * Without this, the robot talks to the hub for EVERY motor.get/set call (slow loop times).
     * With this, it grabs all data in one chunk per loop (fast loop times).
     */
    public static void setBulkReadAuto(HardwareMap hardwareMap) {
        // Get all hubs (Control Hub + Expansion Hub)
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // Iterate and set mode
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * GET BATTERY VOLTAGE
     * Iterates through all expansion hubs to find the lowest voltage.
     * Useful for voltage compensation.
     */
    public static double getBatteryVoltage(HardwareMap hardwareMap) {
        double result = Double.POSITIVE_INFINITY;
        for (LynxModule sensor : hardwareMap.getAll(LynxModule.class)) {
            // Corrected Import: VoltageUnit lives in navigation package
            double voltage = sensor.getInputVoltage(VoltageUnit.VOLTS);
            if (voltage > 0 && voltage < result) {
                result = voltage;
            }
        }
        return result;
    }

    /**
     * VOLTAGE COMPENSATION
     * Adjusts motor power based on battery level.
     * 12V is the "Standard".
     * If battery is 10V, we need to send MORE power (1.2x) to get the same physical result.
     * @param targetPower The power you WANT to send (0.0 to 1.0)
     * @param currentVoltage The actual battery voltage read from the hub
     * @return Compensated power
     */
    public static double voltageCompensate(double targetPower, double currentVoltage) {
        double nominalVoltage = 12.0;
        // Don't divide by zero if battery is dead/unplugged
        if (currentVoltage < 5) return targetPower;

        return targetPower * (nominalVoltage / currentVoltage);
    }

    /**
     * LINEAR REGRESSION PREDICTOR (y = mx + b)
     * Used for "Guess and Check" tuning of the shooter.
     * @param input The x value (Distance)
     * @param m The slope (Rate of change)
     * @param b The y-intercept (Base value)
     * @return The calculated output (Servo Position or RPM)
     */
    public static double linearPredict(double input, double m, double b) {
        return (input * m) + b;
    }

    /**
     * ANGLE WRAP
     * Ensures angles stay within -180 to 180.
     * Essential if turret math goes crazy.
     */
    public static double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
}