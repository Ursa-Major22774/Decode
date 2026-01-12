package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.resources.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.resources.Utilities;


@Configurable
public class Shooter {

    // Hardware
    private Servo pitchServo;
    private DcMotorEx yawMotor;
    private Servo gateServo;
    private Servo kickerServo;
    private DcMotorEx flywheelMotor;
    private Limelight3A limelight;

    // Controllers
    private VelocityPIDFController flywheelController;
    private PIDFController yawController;


    // Telemetry Variables
    public static int currentYaw = 0;
    public static boolean detectsAprilTag = false;

    // Constants (TUNING REQUIRED)
    public static double GATE_OPEN = 0.35;
    public static double GATE_CLOSED = 0.65;

    // Limelight Shit
    public static double tx;
    public static double ty;
    public static double yawCorrection;
    public static double yawMotorPower = 0.5;
    public static double rawDistance = 65.5;


    // Motor Constants
    // Rev HD Hex Motor = 28 ticks per rev (internal).
    // If you have a gearbox (e.g., 3.7:1), multiply this by gear ratio.

    // Regression Constants for Pitch (y = mx + b)
    public static double PITCH_M = -0.1;  public static double PITCH_B = 0.1;

    // Regression Constants for RPM (y = mx + b)
    public static double RPM_M = -4.0;
    public static double RPM_B = 0;

    // Constant for Yaw Motor
    public static double YAW_M = 0.45;
    public static double SMOOTHING_ALPHA = 0.1;

    // Limelight Mounting math
    private final double CAMERA_HEIGHT_INCHES = 11.248661;
    private final double TARGET_HEIGHT_INCHES = 41.3386; // Height of the bucket/goal
    private final double CAMERA_MOUNT_ANGLE = 5; // Degrees

    // Tracking
    public double targetRPM = 0;
    public double yawPowerPid = 0;
    public static double smoothedDistance = 65;
    public static double pitchCorrection = 0;

    private HardwareMap hMap;
    private LookUpTables ballistics;

    public static double KICK_POSITION = -0.1;
    public static double KICK_RESET_POSITION = 0.65;


    public Shooter(HardwareMap hardwareMap) {
        hMap = hardwareMap;

        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        yawMotor = hardwareMap.get(DcMotorEx.class, "yawMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        kickerServo.setDirection(Servo.Direction.REVERSE);
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize Velocity PIDF 5 (kP, kI, kD, kF)
        // Tune kF first! It does 90% of the work.
        flywheelController = new VelocityPIDFController(0.3, 0, 0, 0.01);
        yawController = new PIDFController(0.3, 0.0, 0.0, 0.01);

        // Start closed
        gateServo.setPosition(GATE_CLOSED);

        ballistics = new LookUpTables(hardwareMap);

    }

    public void init(){
        yawMotor.setTargetPosition(0);
        yawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yawMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        yawMotor.setPower(yawMotorPower);
//        yawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.deleteSnapshots();
        resetLuts();
    }

    /**
     * @param isRed Set true for red alliance. Set false for blue alliance
     */
    public void aimAndReady(boolean isRed) {
        limelight.start();
        LLResult llResult = limelight.getLatestResult();

        // 1. Get Limelight Data
        limelight.pipelineSwitch(isRed ? 0 : 1);

        if (llResult != null && llResult.isValid()) {
            tx = (tx == 0 ? llResult.getTx() : Utilities.lowPassFilter(llResult.getTx(), tx, 0.1));
            ty = llResult.getTy();
            detectsAprilTag = true;

            // 2. Calculate Distance
            double angleToGoalRad = Math.toRadians(CAMERA_MOUNT_ANGLE + ty);
            rawDistance = -(TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRad);

            smoothedDistance = (smoothedDistance == 0 ? rawDistance : Utilities.lowPassFilter(rawDistance, smoothedDistance, SMOOTHING_ALPHA));

            targetRPM = Utilities.linearPredict(smoothedDistance, RPM_M, RPM_B);
        } else {
            detectsAprilTag = false;
            idle();
        }


        // 3. Set Yaw (Horizontal Aim)
        currentYaw = yawMotor.getCurrentPosition();
        yawCorrection = YAW_M * tx;
        yawMotor.setTargetPosition((int) (currentYaw + yawCorrection));
        yawMotor.setPower(yawMotorPower);


        // 4. Set Pitch (Vertical Aim)
//        pitchServo.setPosition(0.15);
        pitchServo.setPosition(ballistics.calculatePitch(smoothedDistance));
        // -3.1 is max
        flywheelMotor.setPower(Utilities.voltageCompensate(flywheelController.calculate(ballistics.calculateFlywheelSpeed(smoothedDistance), flywheelMotor.getCurrentPosition()), Utilities.getBatteryVoltage(hMap)));

    }
    public void shoot () {
        openGate();
        kick();
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


        yawPowerPid = yawController.calculate(yawCorrection, yawMotor.getCurrentPosition());
        yawMotor.setPower(yawPowerPid);
    }

    public void idle () {
        limelight.stop();
        stopFlywheel();
    }

    public void openGate() {
        gateServo.setPosition(GATE_OPEN);
    }

    public void closeGate() {
        gateServo.setPosition(GATE_CLOSED);
    }

    public void kick() {
        kickerServo.setPosition(KICK_POSITION);
    }
    public void resetKick(){
        kickerServo.setPosition(KICK_RESET_POSITION);
    }

    public void stopFlywheel() {
        targetRPM = 0;
        flywheelController.reset(); // Reset integral/prev values when stopping
        flywheelMotor.setPower(0.0);
    }
    public double getFlywheelPower () {return flywheelMotor.getPower();}
    public void increaseHeight () { ballistics.pitchCorrection += 0.02; }
    public void decreaseHeight () { ballistics.pitchCorrection -= 0.02; }
    public void adjustHeight (double pitchCorrection) { ballistics.pitchCorrection += pitchCorrection;}
    public void increaseFlywheelSpeed () {ballistics.pitchCorrection += 0.1; }
    public void decreaseFlywheelSpeed () {ballistics.pitchCorrection -= 0.1; }
    public void adjustFlywheelSpeed (double flywheelSpeedCorrection) { ballistics.flywheelSpeedCorrection += flywheelSpeedCorrection; }
    public void resetLuts () {
        ballistics.pitchCorrection = 0;
        ballistics.flywheelSpeedCorrection = 0;
    }
}