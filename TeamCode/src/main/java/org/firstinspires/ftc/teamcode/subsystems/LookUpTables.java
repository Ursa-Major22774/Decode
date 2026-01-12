package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.util.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.resources.Utilities;


public class LookUpTables {
    InterpLUT flywheelLut = new InterpLUT();
    InterpLUT pitchLut = new InterpLUT();
    HardwareMap hardwareMap;
    public double flywheelSpeedCorrection = 0;
    public double pitchCorrection = 0;

    public LookUpTables (HardwareMap hardwareMap) {
        flywheelLut.add(55, -2.8 + flywheelSpeedCorrection);
        flywheelLut.add(60.7, -2.8 + flywheelSpeedCorrection);
        flywheelLut.add(87.5, -2.9 + flywheelSpeedCorrection);
        flywheelLut.add(105.5, -2.9 + flywheelSpeedCorrection);
        flywheelLut.add(130, -2.9 + flywheelSpeedCorrection);
        flywheelLut.add(220, -2.9 + flywheelSpeedCorrection);
        flywheelLut.add(250, -3 + flywheelSpeedCorrection);

        pitchLut.add(55, 0.45 + pitchCorrection);
        pitchLut.add(60.7, 0.48 + pitchCorrection);
        pitchLut.add(87.5, 0.5 + pitchCorrection);
        pitchLut.add(105.5, 0.62 + pitchCorrection);
        pitchLut.add(130, 0.72 + pitchCorrection);
        pitchLut.add(220, 1.15 + pitchCorrection);
        pitchLut.add(250, 1.4 + pitchCorrection);


        flywheelLut.createLUT();
        pitchLut.createLUT();

        this.hardwareMap = hardwareMap;
    }

    public int calculateFlywheelSpeed (double distance) {
        return (int) Utilities.voltageCompensate((Utilities.rpmToTicksPerSec(flywheelLut.get(distance))), Utilities.getBatteryVoltage(hardwareMap));
    }

    public double calculatePitch (double distance) {
        return pitchLut.get(distance);
    }
    public double[] getBallistics (double distance) {
        double interpFlywheelSpeed = Utilities.rpmToTicksPerSec(flywheelLut.get(distance));
        double interpPitch = pitchLut.get(distance);

        double[] collection = {interpFlywheelSpeed, interpPitch};
        return collection;
    }

}
