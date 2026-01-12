package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TurretSubsystem handles the rotation of the turret using a single DcMotorEx.
 * It uses a Limelight 3A for AprilTag tracking and PID control for positioning.
 */
public class Turret {

    // --- Follower for atan2 ---
    private final Follower follower;

    // --- Hardware Objects ---
    private final DcMotorEx yawMotor;
    private final Limelight3A limelight;

    // --- PID Controllers ---
    // One for aiming at a target (based on degrees/tx)
    // One for resetting to zero (based on encoder ticks)
    private final PIDFController visionPID;
    private final PIDFController positionPID;

    // --- Tunable Constants (Set to 0.0 for initial tuning) ---
    public static double VISION_P = 0.0;
    public static double VISION_I = 0.0;
    public static double VISION_D = 0.0;
    public static double VISION_F = 0.0;

    public static double POS_P = 0.0;
    public static double POS_I = 0.0;
    public static double POS_D = 0.0;
    public static double POS_F = 0.0;

    /** Conversion factor: how many encoder ticks are in one degree of turret rotation */
    public static double TICKS_PER_DEGREE = 0.0;

    private double currentPos;
    private double tx;

    /**
     * Constructor for the TurretSubsystem.
     * @param hardwareMap The hardware map from the OpMode.
     * @param follower PedroPathing follower as fallback auto aim.
     */
    public Turret(HardwareMap hardwareMap, Follower follower) {
        // Initialize Motor
        yawMotor = hardwareMap.get(DcMotorEx.class, "yawMotor");
        yawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yawMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize PID Controllers
        visionPID = new PIDFController(VISION_P, VISION_I, VISION_D, VISION_F);
        positionPID = new PIDFController(POS_P, POS_I, POS_D, POS_F);

        // Initialize Follower
        this.follower = follower;

        // Start Limelight
        limelight.start();
    }

    /**
     * Zeros the motor encoder to the current physical position.
     * Use this when the turret is manually centered.
     */
    public void zeroEncoder() {
        yawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Points the turret back to the zero position using the encoder.
     * This is used as a fallback when no target is seen.
     */
    public void reset() {
        limelight.pause();
        currentPos = yawMotor.getCurrentPosition();
        // Target is 0 ticks (center)
        double power = positionPID.calculate(currentPos, 0);
        setSafePower(power);
    }

    /**
     * Primary aim method using Limelight vision data.
     * @param isRed If true, switches to pipeline 0 (Red). If false, switches to pipeline 1 (Blue).
     */
    public void aim(boolean isRed) {
        // 1. Ensure Limelight is active
        limelight.start();

        // 2. Switch pipeline based on alliance
        limelight.pipelineSwitch(isRed ? 0 : 1);

        // 3. Fetch latest result
        currentPos = yawMotor.getCurrentPosition();
        LLResult result = limelight.getLatestResult();

        // 4. Check if a target (AprilTag) is valid
        if (result != null && result.isValid()) {
            tx = result.getTx(); // Angle offset in degrees

            // Aiming target is tx = 0 (the center of the camera)
            // Note: tx is already the error, so we calculate based on 0
            double power = visionPID.calculate(tx, 0);
            setSafePower(power);
        } else {
            // ODOMETRY FALLBACK
            double targetX = isRed ? 137.0 : 0.0;
            double targetY = 143.0;

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeading = follower.getPose().getHeading(); // Radians

            // 1. Calculate field-centric angle to target (Y first!)
            double fieldAngle = Math.atan2(targetY - robotY, targetX - robotX);

            // 2. Subtract robot heading to get robot-relative angle
            double relativeRadians = fieldAngle - robotHeading;

            // 3. Normalize the angle (keep it between -PI and PI)
            relativeRadians = AngleUnit.normalizeRadians(relativeRadians);

            double targetTicks = radiansToTicks(relativeRadians);
            double power = positionPID.calculate(currentPos, targetTicks);
            setSafePower(power);
        }
    }

    /**
     * Helper method to apply motor power while respecting hardware limits.
     * @param power The requested power from the PID controllers.
     */
    private void setSafePower(double power) {
        // --- Soft Limits (Degrees) ---
        final double LEFT_LIMIT_DEG = -90.0;
        final double RIGHT_LIMIT_DEG = 75.0;

        double currentAngle = yawMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        // Prevent rotating past the 90 degree left limit
        if (currentAngle <= LEFT_LIMIT_DEG && power < 0) {
            yawMotor.setPower(0);
        }
        // Prevent rotating past the 75 degree right limit
        else if (currentAngle >= RIGHT_LIMIT_DEG && power > 0) {
            yawMotor.setPower(0);
        }
        else {
            yawMotor.setPower(power);
        }

        if (Math.abs(power) < 0.02) {
            yawMotor.setPower(0);
        }
    }

    public double radiansToTicks (double radians) {
        double degrees = Math.toDegrees(radians);
        return degrees * TICKS_PER_DEGREE;
    }

    public double getTx () {
        return tx;
    }

    public double getCurrentPosition () {
        return currentPos;
    }
}