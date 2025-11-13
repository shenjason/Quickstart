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
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.PIDcontroller;
import org.firstinspires.ftc.teamcode.util.Sequencer;

public class Shooter extends Assembly {
    public double flywheelP = 2, flyWheelI = 0.005, flyWheelD = 1;
    public double turret_yawP = 0.05d;

    final int SAMPLE_T = 100;

    final double TARGET_CENTER_X = 0, TARGET_CENTER_CLEARANCE = 3d;
    final double PITCH_MIN_POS = 0.8, PITCH_MAX_POS = 0.1;
    final double TOP_GATE_IN_POS = 0.5, TOP_GATE_OUT_POS = 0.7;
    final double BOTTOM_OPEN_GATE_POS = 0.65, BOTTOM_CLOSE_GATE_POS = 0.9;


    public final static int GPP = 0, PGP = 1, PPG = 2, BLUE_TARGET_LINE = 3, RED_TARGET_LINE = 4;

    DcMotor flywheelMotor, bootkickerMotor, turretYawEncoder;
    CRServo turretYawServo;
    Servo turretPitchServo, gateServo, bootkickerServo;

    ColorSensor outtakeColorSensor;

    Limelight3A limelight;

    Timer flywheelRPMSampleTimer;

    double flywheelRPM = 0, prevFlyWheelPos = 0, targetFlyWheelRPM = 0;
    public double TagSize = 0;

    public PIDcontroller flywheelPID;

    public Sequencer shooterSequence;


    public Shooter(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
    }

    @Override
    public void hardwareInit() {
        flywheelMotor = hardwareMap.get(DcMotor.class, "shootermotor");
        bootkickerMotor = hardwareMap.get(DcMotor.class, "intakemotor");
        turretYawServo = hardwareMap.get(CRServo.class, "yawservo");
        turretPitchServo = hardwareMap.get(Servo.class, "pitchservo");
        bootkickerServo = hardwareMap.get(Servo.class, "topgate");
        gateServo = hardwareMap.get(Servo.class, "bottomgate");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        outtakeColorSensor = hardwareMap.get(ColorSensor.class, "outakesensor");


        turretYawEncoder = hardwareMap.get(DcMotor.class, "fr");

        turretYawEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretYawServo.setDirection(DcMotorSimple.Direction.REVERSE);





        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelPID = new PIDcontroller(flywheelP, flyWheelI, flyWheelD, 0, 1);

        limelight.setPollRateHz(30);

        limelight.start();

        flywheelRPMSampleTimer = new Timer();


        TopGateIN();
        closeBottomGate();

        shooterSequence = new Sequencer(List.of(
                this::BootkickerON,
                this::TopGateIN,
                this::openBottomGate,
                this::TopGateOUT,
                this::closeBottomGate,
                this::TopGateIN,
                this::BootkickerOFF
        ), List.of(
                0d, 0d, 0d, 0.15d, 0.1d, 0.2d, 0.2d
        ), List.of(
                Sequencer.defaultCondition(),
                Sequencer.defaultCondition(),
                Sequencer.defaultCondition(),
                () -> !isBall(),
                Sequencer.defaultCondition(),
                Sequencer.defaultCondition(),
                Sequencer.defaultCondition()

        ));

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

    public void setPitch(double pitchAngle){
        if (pitchAngle < 45 || pitchAngle > 60) return;
        double perc = (pitchAngle - 45d) / 15d;
        turretPitchServo.setPosition(PITCH_MIN_POS +  (PITCH_MAX_POS - PITCH_MIN_POS) * perc);
    }


    void autoTargeting(){
        if (side) limelight.pipelineSwitch(BLUE_TARGET_LINE);
        else limelight.pipelineSwitch(RED_TARGET_LINE);

        LLResult result = limelight.getLatestResult();
        double offset = turretYawEncoder.getCurrentPosition() * 0.002d;
        if (limelightResultVaild(result) && Math.abs(offset) < 10){
            TagSize = result.getTa();
            debugAddData("targetTagSize", TagSize);

            offset = result.getTx() - TARGET_CENTER_X;
            debugAddData("targetOffsetX", offset);

            debugAddData("targetOffsetABS", Math.abs(offset));

        }

        if (Math.abs(offset) > TARGET_CENTER_CLEARANCE){
            turretYawServo.setPower(Math.max(Math.abs(offset) * turret_yawP, 0.1) * Math.signum(offset));
            debugAddLine("Adjusting...");
        }else{
            turretYawServo.setPower(0);
        }

    }


    public void openBottomGate(){ gateServo.setPosition(BOTTOM_OPEN_GATE_POS);}
    public void closeBottomGate(){ gateServo.setPosition(BOTTOM_CLOSE_GATE_POS);}
    public void TopGateIN(){ bootkickerServo.setPosition(TOP_GATE_IN_POS);}
    public void TopGateOUT(){ bootkickerServo.setPosition(TOP_GATE_OUT_POS);}

    public void BootkickerON(){ bootkickerMotor.setPower(1);}
    public void BootkickerOFF(){ bootkickerMotor.setPower(0);}

    public boolean isGreen(){
        return (outtakeColorSensor.green() > 100);
    }

    public boolean isPurple(){
        return (outtakeColorSensor.blue() > 100 && outtakeColorSensor.red() > 80);
    }

    public boolean isBall(){
        return (isGreen() || isPurple());
    }


    public void autoAdjustShooterParameters(){
        setFlywheelRPM(4000);
        setPitch(60);
    }

    public void offShooter(){
        setFlywheelRPM(0);
    }



    @Override
    public void update() {
        calcFlyWheelRPM();
        debugAddData("flyWheelRPM", flywheelRPM);
        debugAddData("RPMError", flywheelPID.getE());
        debugAddData("flywheelPowerOutput", flywheelPID.currentOutput);
        debugAddData("outtakeColorR", outtakeColorSensor.red());
        debugAddData("outtakeColorG", outtakeColorSensor.green());
        debugAddData("outtakeColorB", outtakeColorSensor.blue());
        debugAddData("outtakeBall", isBall());
        autoTargeting();

        shooterSequence.update();
    }

}
