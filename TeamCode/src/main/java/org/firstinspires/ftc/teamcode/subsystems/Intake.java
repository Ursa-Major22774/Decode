package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {
    private DcMotorEx intakeMotor;



    // Reverse Intake Direction

    // Constant power. Adjust if it's too fast/slow.
    public static double INTAKE_POWER = -0.5;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); //expansion hub 0
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void outtake() {
        // In case we grab the wrong color sample
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}