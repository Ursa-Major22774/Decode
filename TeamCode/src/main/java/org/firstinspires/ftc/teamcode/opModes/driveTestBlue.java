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
@Configurable
@TeleOp(name = "Drive Test (Blue)", group = "Competition OpModes")
public class driveTestBlue extends OpMode {

    // Subsystems
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

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
        shooter = new Shooter(hardwareMap);
//        gateServo.setDirection(Servo.Direction.REVERSE);

        // 3. Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        // 4. Initialize Turret
        shooter.init();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        // --- 1. DRIVETRAIN (PedroPathing) ---4
        // Stick Y is inverted (Up is negative on standard gamepads)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_x, // Forward/Back
                gamepad1.left_stick_y, // Strafe
                -gamepad1.right_stick_x * 0.6, // Turn
                true // TRUE = Robot Centric
        );
        follower.update();
        follower.getPose().getX();


        // --- 2. INTAKE LOGIC ---x
        // Left Trigger = Intake
        if (gamepad1.left_trigger > 0.1) {
            intake.intake();
            transfer.lift();
        } else {
            intake.stop();
            transfer.stop();
        }

        if (gamepad1.a) {
            shooter.aimAndReady(false);
            shooter.update(Utilities.getBatteryVoltage(hardwareMap));
        } else {
            shooter.idle();
        }

        if (gamepad1.right_trigger > 0.1) {
            shooter.shoot();
        } else {
            shooter.resetKick();
            shooter.closeGate();
        }

        if (gamepad1.dpad_up) {
            shooter.increaseHeight();
        } else if (gamepad1.dpad_down) {
            shooter.decreaseHeight();
        }

        if (gamepad1.dpad_right) {
            shooter.increaseFlywheelSpeed();
        } else if (gamepad1.dpad_left) {
            shooter.decreaseFlywheelSpeed();
        }

        if (gamepad1.y) {
            shooter.resetLuts();
        }

        //shooter.update(Utilities.getBatteryVoltage(hardwareMap));

//        if (gamepad1.y){
//            transfer.kick();
//        } else {
//            transfer.resetKick();
//        }

        // Telemetry
        telemetry.addLine("Use Dpad left and right to adjust gate position");
        telemetry.addData("Current Yaw", shooter.currentYaw);
        telemetry.addData("Tx", shooter.tx);
        telemetry.addData("Ty", shooter.ty);
        telemetry.addData("Yaw Correction", shooter.yawCorrection);
        telemetry.addData("Detects April Tag", shooter.detectsAprilTag);
        telemetry.addData("Distance From Target", shooter.rawDistance);
        telemetry.addData("Target RPM", shooter.targetRPM);
        telemetry.addData("Motor Power", shooter.getFlywheelPower());
        telemetry.addData("Yaw Power Pid", shooter.yawPowerPid);
//        telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("State", "Running");
        telemetry.addData("Flywheel Target", "See Dashboard");
        telemetry.update(); telemetryManager.update();
    }
}
