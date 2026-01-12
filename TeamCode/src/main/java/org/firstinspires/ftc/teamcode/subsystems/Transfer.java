package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Transfer {
    private final DcMotorEx transferMotor;
    private final Servo kickerServo;

    public static double LIFT_POWER = -1.0;
    public static double KICK_POSITION = -0.1;
    public static double KICK_RESET_POSITION = 0.65;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        kickerServo.setDirection(Servo.Direction.REVERSE);
    }

    public void lift() {
        transferMotor.setPower(LIFT_POWER);
    }
    public void stop() {
        transferMotor.setPower(0);
    }
    public void kick() {
        kickerServo.setPosition(KICK_POSITION);
    }
    public void resetKick(){
        kickerServo.setPosition(KICK_RESET_POSITION);
    }


}