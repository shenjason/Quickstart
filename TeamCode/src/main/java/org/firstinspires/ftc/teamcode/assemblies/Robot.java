package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.Sequencer;

import java.util.List;

public class Robot extends Assembly {

    public Shooter shooter;



    public Robot(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        shooter = new Shooter(_hardwareMap, _t, _debug, _side);

    }


    @Override
    public void hardwareInit() { }


    @Override
    public void update() {

        shooter.update();
    }
}
