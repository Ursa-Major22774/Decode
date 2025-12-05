package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.resources.Utilities.getBatteryVoltage;

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
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@TeleOp(name = "Drive Test", group = "Testing")
public class driveTest extends OpMode {

    // Subsystems
    private Intake intake;
    private Transfer transfer;
//    private ColorRangeSensor colorSensor;

    private Servo gateServo;
    private double servoPosition = 0;

    private DcMotorEx flywheel;

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
//        colorSensor = hardwareMap.get(ColorRangeSensor.class, "topColorSensor");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gateServo = hardwareMap.get(Servo.class, "gateServo");
//        gateServo.setDirection(Servo.Direction.REVERSE);

        // 3. Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
        gateServo.setPosition(0);
    }

    @Override
    public void loop() {
        // --- 1. DRIVETRAIN (PedroPathing) ---4
        // Stick Y is inverted (Up is negative on standard gamepads)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y, // Forward/Back
                -gamepad1.left_stick_x, // Strafe
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
        if (gamepad1.b) {
            gateServo.setPosition(servoClosedPosition);
        } else if (gamepad1.a) {
            gateServo.setPosition(servoOpenPosition);
        }
        servoPosition = gateServo.getPosition();

        // Flywheel Logic
        if (gamepad1.right_trigger > 0.1) {
            flywheel.setPower(-1);
        } else {
            flywheel.setPower(0);
        }

        // Telemetry
        telemetry.addLine("Use Dpad left and right to adjust gate position");
        telemetry.addData("Servo Postion", servoPosition);
        telemetry.addData("State", "Running");
        telemetry.addData("Flywheel Target", "See Dashboard");
        telemetry.update(); telemetryManager.update();
    }
}
