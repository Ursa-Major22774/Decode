package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    private RevColorSensorV3 colorSensor;
    public ColorSensor(HardwareMap hardwaremap){
        colorSensor = hardwaremap.get(RevColorSensorV3.class, "colorSensor");
    }

    public boolean detectsBall () {
        return (colorSensor.getDistance(DistanceUnit.MM) < 30);
    }
}
