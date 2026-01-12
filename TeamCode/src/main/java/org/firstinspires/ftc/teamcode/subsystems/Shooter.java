package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.resources.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.resources.Utilities;


@Configurable
public class Shooter {

    // Hardware
    private final Servo pitchServo;
    private final DcMotorEx flywheelMotor;
    private final Limelight3A limelight;
    private final Follower follower;

    // Controllers
    private final VelocityPIDFController flywheelController;

    // Telemetry Variables
    public static boolean detectsAprilTag = false;

    // Limelight Shit
    public static double ty;
    public static double rawDistance = 65.5;

    // Tracking
    public double targetRPM = 0;
    public static double smoothedDistance = 65;
    private final LookUpTables ballistics;

    public Shooter(HardwareMap hardwareMap, Follower follower) {
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize Velocity PIDF 5 (kP, kI, kD, kF)
        // Tune kF first! It does 90% of the work.
        flywheelController = new VelocityPIDFController(0.3, 0, 0, 0.01);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ballistics = new LookUpTables(hardwareMap);

        limelight.deleteSnapshots();

        this.follower = follower;

        resetLuts();
    }

    /**
     * @param isRed Set true for red alliance. Set false for blue alliance
     */
    public void accelerateFlywheel (boolean isRed) {
        limelight.start();
        LLResult llResult = limelight.getLatestResult();

        // 1. Get Limelight Data
        limelight.pipelineSwitch(isRed ? 0 : 1);

        if (llResult != null && llResult.isValid()) {
            double CAMERA_MOUNT_ANGLE = 5;
            double CAMERA_HEIGHT_INCHES = 11.248661;
            double TARGET_HEIGHT_INCHES = 41.3386;

            ty = llResult.getTy();
            detectsAprilTag = true;

            // 2. Calculate Distance
            double angleToGoalRad = Math.toRadians(CAMERA_MOUNT_ANGLE + ty);
            rawDistance = -(TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRad);
            smoothedDistance = (smoothedDistance == 0 ? rawDistance : Utilities.lowPassFilter(rawDistance, smoothedDistance, 0.1));

        } else {
            detectsAprilTag = false;

            double lateralDistance = follower.getPose().distanceFrom(new Pose(isRed ? 137 : 7, 143));
            double rawDistance = Math.hypot(lateralDistance, 53.76);
            smoothedDistance = (smoothedDistance == 0 ? rawDistance : Utilities.lowPassFilter(rawDistance, smoothedDistance, 0.1));
        }

        pitchServo.setPosition(ballistics.calculatePitch(smoothedDistance));
        flywheelMotor.setPower(flywheelController.calculate(ballistics.calculateFlywheelSpeed(smoothedDistance), flywheelMotor.getCurrentPosition()));
    }
    public void idle () {
        limelight.deleteSnapshots();
        limelight.pause();
        flywheelController.reset();
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