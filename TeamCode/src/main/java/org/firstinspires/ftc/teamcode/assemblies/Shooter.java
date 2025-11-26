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

    final double OPEN_GATE_POS = 0.65, CLOSE_GATE_POS = 0.9;
    DcMotor flywheelMotor, intakeMotor;
    Servo gateServo;

    Timer flywheelRPMSampleTimer;

    double flywheelRPM = 0, prevFlyWheelPos = 0, targetFlyWheelRPM = 0;

    public PIDcontroller flywheelPID;
    public double TagSize;


    private Turret turret;

    public boolean turret_active = true;


    public void autoAdjustShooterParameters(){
        setFlywheelRPM(TagSize * 0.02d + 60);
    }


    public Shooter(HardwareMap _hardwareMap, Telemetry _t, Follower follower, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        turret = new Turret(hardwareMap, t, follower, debug, side);
    }

    @Override
    public void hardwareInit() {
        flywheelMotor = hardwareMap.get(DcMotor.class, "shooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        gateServo = hardwareMap.get(Servo.class, "gate");


        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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



    public void openGate(){ gateServo.setPosition(OPEN_GATE_POS);}
    public void closeGate(){ gateServo.setPosition(CLOSE_GATE_POS);}

    public void offShooter() {
        setFlywheelRPM(0);
    }

    public void setIntakeMotorPower(double power){
        intakeMotor.setPower(power);
    }


    @Override
    public void update() {
        calcFlyWheelRPM();
        debugAddData("flyWheelRPM", flywheelRPM);
        debugAddData("RPMError", flywheelPID.getE());
        debugAddData("flywheelPowerOutput", flywheelPID.currentOutput);
        autoAdjustShooterParameters();

        if (turret_active) turret.update();
    }

}
