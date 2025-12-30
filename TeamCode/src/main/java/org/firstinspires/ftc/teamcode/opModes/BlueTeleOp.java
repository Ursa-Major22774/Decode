package org.firstinspires.ftc.teamcode.opModes;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Blue TeleOp", group = "Competition TeleOp")
public class BlueTeleOp extends OpMode {

    // Subsystems
    private Intake intake;
    private Transfer transfer;
    private Turret turret;

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
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);

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
                -gamepad1.right_stick_x, // Turn
                true // TRUE = Field Centric
        );
        follower.update();

        // --- 2. INTAKE & TRANSFER LOGIC ---
        // Left Trigger = Intake + Transfer Lift
        if (gamepad1.left_trigger > 0.1) {
            intake.intake();
            transfer.lift();
        }
        // Right Trigger = Shoot + Open Gate
        else if (gamepad1.right_trigger > 0.1) {
            // Keep transfer lifting/feeding to ensure ball hits flywheel
            transfer.feed();
            intake.stop();

            // Run Shoot Sequence
//            turret.shoot(getRuntime());
        }
        // Default State: Hold the ball
        else {
            intake.stop();
            turret.closeGate();
            turret.stopFlywheel();
            // This applies the "Stall" voltage to keep ball from falling
            transfer.hold(getBatteryVoltage(hardwareMap));
        }

        // --- 3. TURRET AIMING ---
        // Button A = Aim/Spin up
        if (gamepad1.a) {
            turret.aimAndReady(false);
        } else {
            // Idle behavior
            turret.stopFlywheel();
        }

        // --- 4. SYSTEM UPDATES ---
        // Run PID loops
        turret.update(getBatteryVoltage(hardwareMap));

        // Telemetry
        telemetry.addData("State", "Running");
        telemetry.addData("Flywheel Target", "See Dashboard");
        telemetry.update(); telemetryManager.update();
    }
}