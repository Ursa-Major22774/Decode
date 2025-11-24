package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.resources.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.resources.Utilities;

public class Turret {

    // Hardware
    private Servo pitchServo;
    private Servo yawServo;
    private Servo gateServo;
    private DcMotorEx flywheelMotor;
    private Limelight3A limelight;

    // Controllers
    private VelocityPIDFController flywheelController;

    // Constants (TUNING REQUIRED)
    private final double GATE_OPEN = 0.5;
    private final double GATE_CLOSED = 0.0;

    // Motor Constants
    // Rev HD Hex Motor = 28 ticks per rev (internal).
    // If you have a gearbox (e.g., 3.7:1), multiply this by gear ratio.
    private final double TICKS_PER_REV = 28.0;

    // Regression Constants for Pitch (y = mx + b)
    private final double PITCH_M = 0.002;
    private final double PITCH_B = 0.1;

    // Regression Constants for RPM (y = mx + b)
    private final double RPM_M = 10.5;
    private final double RPM_B = 1000;

    // Limelight Mounting math
    private final double CAMERA_HEIGHT_INCHES = 10.0;
    private final double TARGET_HEIGHT_INCHES = 25.0; // Height of the bucket/goal
    private final double CAMERA_MOUNT_ANGLE = 20.0; // Degrees

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
        LLResult llResult = limelight.getLatestResult();
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

    /**
     * UPDATE LOOP
     * Must be called every cycle to keep the flywheel PID running.
     */
    public void update(double currentVoltage) {
        // CHANGE: Get Position instead of Velocity
        double currentTicks = flywheelMotor.getCurrentPosition();

        // Convert Target RPM to Target Ticks-Per-Second
        double targetTPS = rpmToTicksPerSec(targetRPM);

        // Calculate power needed
        double power = flywheelController.calculate(targetTPS, currentTicks);

        // Apply voltage compensation
        double safePower = Utilities.voltageCompensate(power, currentVoltage);

        flywheelMotor.setPower(safePower);
    }

    private double rpmToTicksPerSec(double rpm) {
        // RPM / 60 = Revolutions Per Second
        // RPS * TicksPerRev = Ticks Per Second
        return (rpm / 60.0) * TICKS_PER_REV;
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