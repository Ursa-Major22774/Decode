package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.resources.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.resources.Utilities;
import org.slf4j.helpers.Util;

import java.util.Timer;
import java.util.TimerTask;

@Configurable
public class Turret {

    // Hardware
    private Servo pitchServo;
    private Servo yawServo;
    private Servo gateServo;
    private DcMotorEx flywheelMotor;
    private Limelight3A limelight;

    // Controllers
    private VelocityPIDFController flywheelController;

    // --- TIMING/STATE VARIABLES ---
    private double gateOpenedTime = 0; // Stores the time (in seconds) the gate was opened
    private ShootingState currentShootingState = ShootingState.IDLE;
    private final double SHOOT_DELAY_SECONDS = 0.5; // 500 ms delay

    public enum ShootingState {
        IDLE,             // Ready to spin up
        SPINNING_UP,      // Waiting for flywheel to hit target RPM
        FEEDING,          // Gate is open, transferring balls
        RELOAD            // Gate closing, waiting to go back to IDLE
    }

    // Constants (TUNING REQUIRED)
    private final double GATE_OPEN = 0.5;
    private final double GATE_CLOSED = 0.0;

    // Motor Constants
    // Rev HD Hex Motor = 28 ticks per rev (internal).
    // If you have a gearbox (e.g., 3.7:1), multiply this by gear ratio.

    // Regression Constants for Pitch (y = mx + b)
    private static double PITCH_M = 0.002;
    private static double PITCH_B = 0.1;

    // Regression Constants for RPM (y = mx + b)
    private static double RPM_M = 10.5;
    private static double RPM_B = 1000;

    // Limelight Results
    LLResult llResult;

    // Limelight Mounting math
    private final double CAMERA_HEIGHT_INCHES = 11.097;
    private final double TARGET_HEIGHT_INCHES = 41.3386; // Height of the bucket/goal
    private final double CAMERA_MOUNT_ANGLE = 12.261; // Degrees

    // Tracking
    private double targetRPM = 0;

    public Turret(HardwareMap hardwareMap) {
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        yawServo = hardwareMap.get(Servo.class, "yawServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize Velocity PIDF (kP, kI, kD, kF)
        // Tune kF first! It does 90% of the work.
        flywheelController = new VelocityPIDFController(0.0, 0.0, 0.0, 1.3);

        // Start closed
        gateServo.setPosition(GATE_CLOSED);
    }

    /**
     * AIM AND READY
     * 1. Read Limelight tx/ty.
     * 2. Calculate Distance.
     * 3. Set Servos.
     * 4. Set Flywheel RPM.
     */
    public void aimAndReady(String allianceColor) {
        double tx = 0;
        double ty = 0;

        // 1. Get Limelight Data
        if (allianceColor == "red") {
            limelight.pipelineSwitch(0);
        } else if (allianceColor == "blue") {
            limelight.pipelineSwitch(1);
        }

        if (llResult != null && llResult.isValid()) {
            tx = llResult.getTx();
            ty = llResult.getTy();
        }

        // 2. Calculate Distance
        double angleToGoalRad = Math.toRadians(CAMERA_MOUNT_ANGLE + ty);
        double distance = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRad);

        // 3. Set Yaw (Horizontal Aim)
        double currentYaw = yawServo.getPosition();
        double yawCorrection = tx * 0.01;
        yawServo.setPosition(currentYaw + yawCorrection);

        // 4. Set Pitch (Vertical Aim)
        double newPitch = Utilities.linearPredict(distance, PITCH_M, PITCH_B);
        pitchServo.setPosition(newPitch);

        // 5. Set Flywheel Speed
        targetRPM = Utilities.linearPredict(distance, RPM_M, RPM_B);
    }

    public boolean shoot(double currentTime) {
        double currentRPM = Utilities.getRpm(flywheelMotor);
        boolean isFlywheelReady = Math.abs(targetRPM - currentRPM) <= 50;

        switch (currentShootingState) {
            case IDLE:
            case SPINNING_UP:
                // We stay in SPINNING_UP until the flywheel is ready
                if (isFlywheelReady && targetRPM > 0) {
                    // Transition to feeding (open gate)
                    currentShootingState = ShootingState.FEEDING;
                    openGate();
                    gateOpenedTime = currentTime; // Start the 500ms timer
                }
                break;

            case FEEDING:
                // Check if 500ms has elapsed since the gate opened
                if (currentTime >= gateOpenedTime + SHOOT_DELAY_SECONDS) {
                    closeGate();
                    currentShootingState = ShootingState.RELOAD;
                }
                break;

            case RELOAD:
                // Give a moment for the servo to finish closing, then stop flywheel
                // Note: We don't really need a second delay here, but we could add one.
                currentShootingState = ShootingState.IDLE;
                stopFlywheel(); // Reset target RPM to 0
                return false; // Sequence finished

            default:
                // Should not happen
                currentShootingState = ShootingState.IDLE;
                break;
        }

        return currentShootingState != ShootingState.IDLE;
    }

    /**
     * UPDATE LOOP
     * Must be called every cycle to keep the flywheel PID running.
     */
    public void update(double currentVoltage) {
        // Get Position
        double currentTicks = flywheelMotor.getCurrentPosition();

        // Convert Target RPM to Target Ticks-Per-Second
        double targetTPS = Utilities.rpmToTicksPerSec(targetRPM);

        // Calculate power needed
        double power = flywheelController.calculate(targetTPS, currentTicks);

        // Apply voltage compensation
        double safePower = Utilities.voltageCompensate(power, currentVoltage);

        llResult = limelight.getLatestResult();

        flywheelMotor.setPower(safePower);
    }

    public void openGate() {
        gateServo.setPosition(GATE_OPEN);
    }

    public void closeGate() {
        gateServo.setPosition(GATE_CLOSED);
    }

    public void stopFlywheel() {
        targetRPM = 0;
        flywheelController.reset(); // Reset integral/prev values when stopping
    }
}