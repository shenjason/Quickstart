package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.PIDcontroller;

public class Shooter extends Assembly {
    public double flywheelP = 2, flyWheelI = 0.005, flyWheelD = 1, TagSize = 0;
    public double turret_yawP = 2, turret_yawI = 0.005, turret_yawD = 0;
    final int SAMPLE_T = 100, TARGET_CENTER_X = 0;


    public final static int PGP = 1;
    public final static int GPP = 0;
    public final static int PPG = 2;
    final int BLUE_TARGET_LINE = 3;
    final int RED_TARGET_LINE = 4;

    DcMotor flywheelMotor, bootkickerMotor, spinnerMotor;
    CRServo turretYawServo;
    Servo turretPitchServo, gateServo, bootkickerServo;

    ColorSensor outtakeColorSensor;

    Limelight3A limelight;

    Timer flywheelRPMSampleTimer;

    double flywheelRPM, prevFlyWheelPos, targetFlyWheelRPM;

    public PIDcontroller flywheelPID, turret_yawPID;


    public Shooter(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
    }

    @Override
    public void hardwareInit() {
        flywheelMotor = hardwareMap.get(DcMotor.class, "shootermotor");
        bootkickerMotor = hardwareMap.get(DcMotor.class, "intakemotor");
        turretYawServo = hardwareMap.get(CRServo.class, "yawservo");
        turretPitchServo = hardwareMap.get(Servo.class, "pitchservo");
        bootkickerServo = hardwareMap.get(Servo.class, "topgateservo");
        gateServo = hardwareMap.get(Servo.class, "bottomgateservo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        spinnerMotor = hardwareMap.get(DcMotor.class, "spinnermotor");
        outtakeColorSensor = hardwareMap.get(ColorSensor.class, "outakesensor");



        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelPID = new PIDcontroller(flywheelP, flyWheelI, flyWheelD, 0, 1);
        turret_yawPID = new PIDcontroller(turret_yawP, turret_yawI, turret_yawD,  -1, 1);

        limelight.setPollRateHz(30);

        limelight.start();

        flywheelRPMSampleTimer = new Timer();
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

        flywheelMotor.setPower(flywheelPID.step(targetFlyWheelRPM / 6000d, flywheelRPM / 6000d));
    }


    public LLResult limelightGetResult(int pipeline_index) {
        limelight.pipelineSwitch(pipeline_index);
        LLResult result = limelight.getLatestResult();
        while (result.getPipelineIndex() != pipeline_index) result = limelight.getLatestResult();

        return result;
    }

    public boolean limelightResultVaild(LLResult result){
        return (result != null && result.isValid());
    }

    public int determinePattern(){
        if (limelightResultVaild(limelightGetResult(PGP))){
            return PGP;
        }
        if (limelightResultVaild(limelightGetResult(GPP))){
            return GPP;
        }
        if (limelightResultVaild(limelightGetResult(PPG))){
            return PPG;
        }
        return -1;
    }


    public void setFlywheelRPM(double rpm){
        targetFlyWheelRPM = rpm;
    }

    public boolean atTargetFlywheelRPM(){
        return Math.abs(targetFlyWheelRPM-flywheelRPM) < 150;
    }


    void autoTargeting(){
        if (side) limelight.pipelineSwitch(BLUE_TARGET_LINE);
        else limelight.pipelineSwitch(RED_TARGET_LINE);

        LLResult result = limelight.getLatestResult();
        if (limelightResultVaild(result)){
            TagSize = result.getTa();
            debugAddData("targetTagSize", TagSize);
            debugAddData("targetOffsetX", result.getTx());





        }

    }


    @Override
    public void update() {
        calcFlyWheelRPM();
        debugAddData("flyWheelRPM", flywheelRPM);
        debugAddData("RPMError", flywheelPID.getE());
        debugAddData("flywheelPowerOutput", flywheelPID.currentOutput);
        autoTargeting();
    }

}
