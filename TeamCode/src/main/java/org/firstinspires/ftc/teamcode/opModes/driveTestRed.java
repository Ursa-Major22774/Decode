package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
@TeleOp(name = "Drive Test (Red)", group = "LM1 OpModes")
public class driveTestRed extends OpMode {

    // Subsystems
    private Intake intake;
    private Transfer transfer;
    private Turret turret;

    // Pedro Pathing Follower (Handles Drivetrain)
    private Follower follower;

    // Telemetry Manager
    private TelemetryManager telemetryManager;

    public static double servoOpenPosition = 0.25;
    public static double servoClosedPosition = 0.55;

    @Override
    public void init() {
        // 1. Optimize Hardware Reads
        Utilities.setBulkReadAuto(hardwareMap);

        // 2. Initialize Subsystems
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
//        gateServo.setDirection(Servo.Direction.REVERSE);

        // 3. Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        // 4. Initialize Turret
        turret.init();
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

        // --- 2. INTAKE LOGIC ---x
        // Left Trigger = Intake
        if (gamepad1.left_trigger > 0.1) {
            intake.intake();
            transfer.lift();
        } else {
            intake.stop();
            transfer.stop();
        }

        // Flywheel Logic
//        if (gamepad1.right_trigger > 0.1) {
//
//        } else {
//
//        }

        if (gamepad1.a) {
            turret.aimAndReady(true);
            turret.update(Utilities.getBatteryVoltage(hardwareMap));
        } else {
            turret.idle();
        }

        if (gamepad1.right_trigger > 0.1) {
            transfer.kick();
        } else {
            transfer.resetKick();
        }

        //turret.update(Utilities.getBatteryVoltage(hardwareMap));

//        if (gamepad1.y){
//            transfer.kick();
//        } else {
//            transfer.resetKick();
//        }

        // Telemetry
        telemetry.addLine("Use Dpad left and right to adjust gate position");
        telemetry.addData("Current Yaw", turret.currentYaw);
        telemetry.addData("Tx", turret.tx);
        telemetry.addData("Ty", turret.ty);
        telemetry.addData("Yaw Correction", turret.yawCorrection);
        telemetry.addData("Detects April Tag", turret.detectsAprilTag);
        telemetry.addData("Distance From Target", turret.rawDistance);
        telemetry.addData("Target RPM", turret.targetRPM);
        telemetry.addData("Motor Power", turret.getFlywheelPower());
        telemetry.addData("Yaw Power Pid", turret.yawPowerPid);
//        telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("State", "Running");
        telemetry.addData("Flywheel Target", "See Dashboard");
        telemetry.update(); telemetryManager.update();
    }
}
