package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.util.*;

import org.firstinspires.ftc.teamcode.resources.Utilities;


public class LookUpTables {
    InterpLUT flywheelLut = new InterpLUT();
    InterpLUT pitchLut = new InterpLUT();

    public void init () {
        flywheelLut.add(6,7);

        pitchLut.add(6,7);

        flywheelLut.createLUT();
        pitchLut.createLUT();
    }
    public double[] getBallistics (double distance) {
        double interpFlywheelSpeed = Utilities.rpmToTicksPerSec(flywheelLut.get(distance));
        double interpPitch = pitchLut.get(distance);

        double[] collection = {interpFlywheelSpeed, interpPitch};
        return collection;
    }

}
