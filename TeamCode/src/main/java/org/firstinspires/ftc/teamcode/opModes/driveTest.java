package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.resources.Utilities.getBatteryVoltage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.resources.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

// PEDRO PATHING IMPORTS
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive Test", group = "Testing")
public class driveTest extends OpMode {

    // Subsystems
    private Intake intake;

//    private DcMotorEx flywheel;

    // Pedro Pathing Follower (Handles Drivetrain)
    private Follower follower;

    // Telemetry Manager
    private TelemetryManager telemetryManager;


    @Override
    public void init() {
        // 1. Optimize Hardware Reads
        Utilities.setBulkReadAuto(hardwareMap);

        // 2. Initialize Subsystems
        intake = new Intake(hardwareMap);
//        flywheel = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 3. Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        // --- 1. DRIVETRAIN (PedroPathing) ---
        // Stick Y is inverted (Up is negative on standard gamepads)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y, // Forward/Back
                -gamepad1.left_stick_x, // Strafe
                -gamepad1.right_stick_x * 0.6, // Turn
                false // TRUE = Robot Centric
        );
        follower.update();

        // --- 2. INTAKE LOGIC ---
        // Left Trigger = Intake
        if (gamepad1.left_trigger > 0.1) {
            intake.intake();
        } else {
            intake.stop();
        }

        // Flywheel Logic
//        if (gamepad1.right_trigger > 0.1) {
//            flywheel.setPower(1);
//        } else {
//            flywheel.setPower(0);
//        }

        // Telemetry
        telemetry.addData("State", "Running");
        telemetry.addData("Flywheel Target", "See Dashboard");
        telemetry.update(); telemetryManager.update();
    }
}
