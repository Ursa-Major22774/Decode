package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.resources.Utilities;

@Configurable
public class Transfer {
    private DcMotorEx transferMotor;

    // HIGH POWER: To lift the ball from intake to gate
    public static double LIFT_POWER = 1.0;

    // HOLD POWER: The "Stall" power.
    // DANGER: Must be tuned carefully.
    // Too High = Burn motor. Too Low = Ball falls.
    // Start at 0.1 and work up slowly.
    public static double HOLD_POWER = 0.15;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
}