package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.resources.Utilities;

@Configurable
public class Transfer {
    private DcMotorEx transferMotor;
    private Servo kickerServo;

    // HIGH POWER: To lift the ball from intake to gate
    public static double LIFT_POWER = -1.0;

    // HOLD POWER: The "Stall" power.
    // DANGER: Must be tuned carefully.
    // Too High = Burn motor. Too Low = Ball falls.
    // Start at 0.1 and work up slowly.
    public static double HOLD_POWER = -0.1;

    public static double KICK_POSITION = -0.1;
    public static double RESET_POSITION = 0.65;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        kickerServo.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * LIFT
     * Called when the intake is running.
     * Moves ball up to the gate.
     */
    public void lift() {
        transferMotor.setPower(LIFT_POWER);
    }

    /**
     * HOLD
     * Called when intake is released.
     * Applies small voltage to keep ball pinned against the closed gate.
     * Includes Voltage Compensation to ensure consistent holding force
     * regardless of battery level.
     */
    public void hold(double currentVoltage) {
        // Compensate so 10V battery doesn't drop the ball
        double compensatedPower = Utilities.voltageCompensate(HOLD_POWER, currentVoltage);
        transferMotor.setPower(compensatedPower);
    }

    /**
     * FEED
     * Called when we are shooting.
     * Pushes the ball into the flywheel.
     */
    public void feed() {
        transferMotor.setPower(LIFT_POWER);
    }

    public void stop() {
        transferMotor.setPower(0);
    }

    public void kick() {
        kickerServo.setPosition(KICK_POSITION);
    }
    public void resetKick(){
        kickerServo.setPosition(RESET_POSITION);
    }
}