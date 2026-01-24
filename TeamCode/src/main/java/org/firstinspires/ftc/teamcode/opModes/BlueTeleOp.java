package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.resources.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

// PEDRO PATHING IMPORTS
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
@TeleOp(name = "Blue TeleOp", group = "Competition OpModes")
public class BlueTeleOp extends OpMode {

    // Subsystems
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;

    // Pedro Pathing Follower (Handles Drivetrain)
    private Follower follower;

    // Telemetry Manager
    private TelemetryManager telemetryManager;

    @Override
    public void init() {
        // Initialize Follower
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        // Optimize Hardware Reads
        Utilities.setBulkReadAuto(hardwareMap);

        // Initialize Subsystems
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, follower);
        turret = new Turret(hardwareMap, follower);

        // Initialize Telemetry Manager
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
                -gamepad1.left_stick_x, // Forward/Back
                +gamepad1.left_stick_y, // Strafe
                -gamepad1.right_stick_x * 0.6, // Turn
                true // TRUE = Robot Centric
        );
        follower.update();

        // --- 2. INTAKE LOGIC ---
        // Left Trigger = Intake
        if (gamepad1.left_trigger > 0.1) {
            intake.intake();
            transfer.lift();
        } else {
            intake.stop();
            transfer.stop();
        }

        // --- 3. Prepare Shot ---
        if (gamepad2.right_bumper) {
            turret.aim(false);
            shooter.accelerateFlywheel(false);
        } else {
            shooter.idle();
            turret.reset();
        }

        // --- 3. Shoot ---
        if (gamepad2.right_trigger > 0.1) {
            transfer.kick();
        } else {
            transfer.resetKick();
        }

        // --- Adjust Shooting ---
        if (gamepad2.dpad_up) {
            shooter.increaseHeight();
        } else if (gamepad2.dpad_down) {
            shooter.decreaseHeight();
        }

        if (gamepad2.dpad_right) {
            shooter.increaseFlywheelSpeed();
        } else if (gamepad2.dpad_left) {
            shooter.decreaseFlywheelSpeed();
        }

        if (gamepad1.y) {
            shooter.resetLuts();
        }

        // --- Reset Yaw Encoder ---
        if (gamepad1.b) {
            turret.zeroEncoder();
        }


        // Telemetry
        telemetry.addLine("Use Dpad left and right to adjust gate position");
        telemetry.addData("Current Yaw", turret.getCurrentPosition());
        telemetry.addData("Tx", turret.getTx());
        telemetry.addData("Ty", shooter.ty);
        telemetry.addData("Detects April Tag", shooter.detectsAprilTag);
        telemetry.addData("Distance From Target", shooter.rawDistance);
        telemetry.addData("Target RPM", shooter.targetRPM);
        telemetry.addData("Flywheel Power", shooter.getFlywheelPower());
//        telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("State", "Running");
        telemetry.addData("Flywheel Target", "See Dashboard");
        telemetry.update(); telemetryManager.update();
    }
}
