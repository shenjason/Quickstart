package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.PIDcontroller;
import org.firstinspires.ftc.teamcode.util.Sequencer;

public class Shooter extends Assembly {
    public double flywheelP = 2, flyWheelI = 0.005, flyWheelD = 1;
    public double turret_yawP = 0.05d;

    final int SAMPLE_T = 100;

    final double OPEN_GATE_POS = 0.85, CLOSE_GATE_POS = 1;
    final double KICK_BOOT_POS = 0.3, IDLE_BOOT_POS = 0;
    DcMotor flywheelMotor, intakeMotor;
    Servo gateServo, bootKickerServo;

    Timer flywheelRPMSampleTimer;

    double flywheelRPM = 0, prevFlyWheelPos = 0, targetFlyWheelRPM = 0;

    public PIDcontroller flywheelPID;
    public double TagSize;



    public Turret turret;

    public boolean turret_active = true, shooting = false;

    public Sequencer bootSequence = new Sequencer(List.of(
            () -> setBootkickerState(true),
            () -> setBootkickerState(false)
    ), List.of(
            0d, 0.5d
    ));

    public Sequencer shootSequence = new Sequencer(List.of(
            () -> shooting = true,
            this::openGate,
            () -> setIntakeMotorPower(-1),
            () -> bootSequence.start(),
            () -> setIntakeMotorPower(0),
            () -> shooting = false,
            this::closeGate
    ), List.of(
            0d, 0d, 0d, 1.5d, 0.3d, 0d, 0.5d
    ));



    public void autoAdjustShooterParameters(){
        setFlywheelRPM(2800);
    }


    public Shooter(HardwareMap _hardwareMap, Telemetry _t, Follower follower, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        turret = new Turret(hardwareMap, t, follower, debug, side);
    }

    @Override
    public void hardwareInit() {
        flywheelMotor = hardwareMap.get(DcMotor.class, "fly");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        gateServo = hardwareMap.get(Servo.class, "gate");
        bootKickerServo = hardwareMap.get(Servo.class, "boot");


        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelPID = new PIDcontroller(flywheelP, flyWheelI, flyWheelD, 0, 1);

        flywheelRPMSampleTimer = new Timer();


        closeGate();
    }

    void calcFlyWheelRPM(){
        if (flywheelRPMSampleTimer.getElapsedTime() < SAMPLE_T) return;

        if (debug) {
            flywheelPID.p = flywheelP; flywheelPID.i = flyWheelI; flywheelPID.d = flyWheelD;
        }

        flywheelRPM = ((flywheelMotor.getCurrentPosition() - prevFlyWheelPos) / 28d)
                / (flywheelRPMSampleTimer.getElapsedTime() / 60000d);
        flywheelRPMSampleTimer.resetTimer();
        prevFlyWheelPos = flywheelMotor.getCurrentPosition();
        // *0.000167 same as /6000
        flywheelMotor.setPower(flywheelPID.step(targetFlyWheelRPM * 0.000167d, flywheelRPM * 0.000167d));
    }


    public void setFlywheelRPM(double rpm){
        targetFlyWheelRPM = rpm;
    }

    public boolean atTargetFlywheelRPM(){
        return Math.abs(targetFlyWheelRPM-flywheelRPM) < 150;
    }

    public boolean canShoot(){
        return turret.isPointed() && atTargetFlywheelRPM();
    }

    public void Shoot(){
        if (!canShoot()) return;
        shootSequence.start();
    }


    public void openGate(){ gateServo.setPosition(OPEN_GATE_POS);}
    public void closeGate(){ gateServo.setPosition(CLOSE_GATE_POS);}

    public void offShooter() {
        setFlywheelRPM(0);
    }

    public void setIntakeMotorPower(double power){
        intakeMotor.setPower(power);
    }


    public void idleMode(){
        turret.mode = Turret.IDLE_MODE;
    }

    public void trackingMode(){
        turret.mode = Turret.TRACKING_MODE;
    }

    public void setBootkickerState(boolean state){
        double pos = state ? KICK_BOOT_POS : IDLE_BOOT_POS;
        bootKickerServo.setPosition(pos);
    }


    public int turretMode(){
        return turret.mode;
    }


    @Override
    public void update() {
        calcFlyWheelRPM();
        debugAddLine("Shooter");
        debugAddData("flyWheelRPM", flywheelRPM);
        debugAddData("RPMError", flywheelPID.getE());
        debugAddData("flywheelPowerOutput", flywheelPID.currentOutput);

        if (turret_active) turret.update();
        TagSize = turret.Ta;

        shootSequence.update();
        bootSequence.update();
    }

}
