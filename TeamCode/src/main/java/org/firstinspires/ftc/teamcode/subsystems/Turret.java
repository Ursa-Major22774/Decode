package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.resources.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.resources.Utilities;
import org.slf4j.helpers.Util;

import java.util.Timer;
import java.util.TimerTask;

import kotlin.ParameterName;

@Configurable
public class Turret {

    // Hardware
    private Servo pitchServo;
    private DcMotorEx yawMotor;
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

    // Telemetry Variables
    public static int currentYaw = 0;
    public static boolean detectsAprilTag = false;

    // Constants (TUNING REQUIRED)
    public static double GATE_OPEN = 0.25;
    public static double GATE_CLOSED = 0.55;

    // Velocity PIDF Constants
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.2;

    // Limelight Shit
    public static double tx;
    public static double ty;
    public static double yawCorrection;
    public static double yawMotorPower = 0.5;


    // Motor Constants
    // Rev HD Hex Motor = 28 ticks per rev (internal).
    // If you have a gearbox (e.g., 3.7:1), multiply this by gear ratio.

    // Regression Constants for Pitch (y = mx + b)
    public static double PITCH_M = 0.1; //0.002;
    public static double PITCH_B = 0.5; //0.1;

    // Regression Constants for RPM (y = mx + b)
    public static double RPM_M = 10.5;
    public static double RPM_B = 1000;

    // Constant for Yaw Motor
    public static double YAW_M = 0.45;

    // Limelight Mounting math
    private final double CAMERA_HEIGHT_INCHES = 11.248661;
    private final double TARGET_HEIGHT_INCHES = 41.3386; // Height of the bucket/goal
    private final double CAMERA_MOUNT_ANGLE = 5; // Degrees

    // Tracking
    private double targetRPM = 0;

    public Turret(HardwareMap hardwareMap) {
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        yawMotor = hardwareMap.get(DcMotorEx.class, "yawMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize Velocity PIDF 5 (kP, kI, kD, kF)
        // Tune kF first! It does 90% of the work.
        flywheelController = new VelocityPIDFController(kP, kI, kD, kF);

        // Start closed
        gateServo.setPosition(GATE_CLOSED);
    }

    /**
     * @param isRed Set true for red alliance. Set false for blue alliance
     */

    public void aimAndReady(boolean isRed) {
        yawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yawMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight.start();
        LLResult llResult = limelight.getLatestResult();

        // 1. Get Limelight Data
        limelight.pipelineSwitch(isRed ? 0 : 1);

        if (llResult != null && llResult.isValid()) {
            tx = llResult.getTx();
            ty = llResult.getTy();
            detectsAprilTag = true;
        } else {
            detectsAprilTag = false;
            yawMotor.setTargetPosition(0);
        }

        // 2. Calculate Distance
        double angleToGoalRad = Math.toRadians(CAMERA_MOUNT_ANGLE + ty);
        double distance = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRad);

        // 3. Set Yaw (Horizontal Aim)
        currentYaw = yawMotor.getCurrentPosition();
        yawCorrection = YAW_M * tx;
        yawMotor.setTargetPosition((int) (currentYaw + yawCorrection));
        yawMotor.setPower(yawMotorPower);

        // 4. Set Pitch (Vertical Aim)
//        double newPitch = Utilities.linearPredict(distance, PITCH_M, PITCH_B);
//        pitchServo.setPosition(newPitch);
//
//        // 5. Set Flywheel Speed
//        targetRPM = Utilities.linearPredict(distance, RPM_M, RPM_B);
    }

    /**
     * @param currentTime
     * @return
     */
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