package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.Sequencer;

import java.util.List;

public class Robot extends Assembly {
    public double GREEN = 0.45, BLUE = 0.611, RED = 0.28;
    public Shooter shooter;

    public Servo led;
    public DistanceSensor distanceSensor;

    public Robot(HardwareMap _hardwareMap, Telemetry _t, Follower f, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        shooter = new Shooter(_hardwareMap, _t, f, _debug, _side);

        idle();
        intake(false);
    }

    public void shoot(){
        if (shooter.targetFlyWheelRPM <= 2000) return;
        shooter.Shoot();
    }

    public void idle(){
        shooter.idleMode();
        shooter.setFlywheelRPM(0);
    }

    public void tracking(){
        shooter.trackingMode();
        shooter.autoAdjustShooterParameters();
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
    public void hardwareInit() {
        led = hardwareMap.get(Servo.class, "light");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        led.setPosition(0);
    }


    @Override
    public void update() {

        shooter.update();

        if (shooter.turret.mode == Turret.TRACKING_MODE){
            if (shooter.atTargetFlywheelRPM()){
                if (shooter.turret.isPointed()){
                    led.setPosition(GREEN);
                }else{
                    led.setPosition(BLUE);
                }
            }else{
                led.setPosition(RED);
            }
        }else{
            if (distanceSensor.getDistance(DistanceUnit.MM) < 80){
                led.setPosition(GREEN);
            }else{
                led.setPosition(RED);
            }

        }
    }
}
