package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.util.*;

import org.firstinspires.ftc.teamcode.resources.Utilities;


public class LookUpTables {
    InterpLUT flywheelLut = new InterpLUT();
    InterpLUT pitchLut = new InterpLUT();
    public LookUpTables () {
        flywheelLut.add(55, -3);
        flywheelLut.add(60.7, -3);
        flywheelLut.add(87.5, -3.1);
        flywheelLut.add(105.5, -3.1);
        flywheelLut.add(130, -3.1);
        flywheelLut.add(220, -3.1);

        pitchLut.add(55, 0.5);
        pitchLut.add(60.7, 0.53);
        pitchLut.add(87.5, 0.55);
        pitchLut.add(105.5, 0.67);
        pitchLut.add(130, 0.77);
        pitchLut.add(220, 1.2);


        flywheelLut.createLUT();
        pitchLut.createLUT();
    }

    public int calculateFlywheelSpeed (double distance) {
        return (int)(Utilities.rpmToTicksPerSec(flywheelLut.get(distance)));
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
