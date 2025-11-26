package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.Sequencer;

import java.util.List;

public class Robot extends Assembly {

    public Shooter shooter;

    public Robot(HardwareMap _hardwareMap, Telemetry _t, Follower f, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        shooter = new Shooter(_hardwareMap, _t, f, _debug, _side);

        idle();
    }

    public void shoot(){
        shooter.Shoot();
    }

    public void idle(){
        shooter.idleMode();
    }

    public void tracking(){
        shooter.trackingMode();
    }
    public void intake(boolean state){
        if (shooter.shooting) return;
        if (state){
            shooter.closeGate();
            shooter.setIntakeMotorPower(-1);
            return;
        }

        shooter.setIntakeMotorPower(0);

    }



    @Override
    public void hardwareInit() { }


    @Override
    public void update() {

        shooter.update();
    }
}
