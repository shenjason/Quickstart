package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.PIDcontroller;
import org.opencv.core.Mat;


public class Turret extends Assembly {
    public double P=2.5,I=0.002,D=0.25,F=0.03;

    public PIDcontroller turretController;
    public boolean isInCamera;
    private double targetRotation;
    public double Ta;
    private double Tx, TagEx;
    private Limelight3A limelight;

    private DcMotor turretMotor;
    private Follower follower;

    private double targetPointX;

    private final double TARGETBLUEX=12, TARGETY=132, TARGETREDX=144-TARGETBLUEX;

    public final static int GPP = 0, PGP = 1, PPG = 2, BLUE_TARGET_LINE = 3, RED_TARGET_LINE = 4, TRACKING_MODE = 0, IDLE_MODE = 1;
    public int mode = IDLE_MODE;
    public double offsetAngle = 0;

    public double debugTargetAngle = 0;

    Timer cameraTTimer;


    public Turret(HardwareMap _hardwareMap, Telemetry _t, Follower f, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        follower = f;
    }

    @Override
    public void hardwareInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight.setPollRateHz(30);

        turretController = new PIDcontroller(P, I, D, F, -0.3, 0.3);

        limelight.pipelineSwitch((side) ? BLUE_TARGET_LINE : RED_TARGET_LINE);
        targetPointX = (side) ? TARGETBLUEX : TARGETREDX;
        limelight.start();

        cameraTTimer = new Timer();
    }


    @Override
    public void update() {

        LLResult llResult = limelight.getLatestResult();
        Pose cp = follower.getPose();
        double angle = getAngle(cp.getX(),cp.getY(),targetPointX,TARGETY);
        double robotAngle = cp.getHeading();
        isInCamera = limelightResultVaild(llResult);

        if (isInCamera && (atTargetPosition() || Tx==0)) {
            Ta = llResult.getTa();
            TagEx = Math.abs(llResult.getTx());
            if (TagEx >= 5) Tx = llResult.getTx();
        }
        if (!isInCamera) {
            Ta = 0; TagEx = 0;
            if (cameraTTimer.getElapsedTime() > 1000) Tx=0;
        }else{
            cameraTTimer.resetTimer();
        }

        double diffAngle = getDiff(angle,robotAngle);
        targetRotation = diffAngle - Math.toRadians(Tx);
        double currentRotation = (turretMotor.getCurrentPosition()/145.1d*0.16d*2d*Math.PI - offsetAngle);

        double clamped_target_rot = targetRotation;
        if (Math.abs(targetRotation)>=Math.toRadians(70)) clamped_target_rot = Math.toRadians(70)*Math.signum(targetRotation);

        turretController.p = P; turretController.i = I; turretController.d = D; turretController.f = F;

        turretMotor.setPower(turretController.step((mode == Turret.TRACKING_MODE) ? -clamped_target_rot : debugTargetAngle, currentRotation));

        debugAddLine("Turret");
        debugAddData("InFrame", isInCamera);
        debugAddData("Mode", mode);
        debugAddData("PoseX", cp.getX());
        debugAddData("PoseY", cp.getY());
        debugAddData("Heading", Math.toDegrees(cp.getHeading()));
        debugAddData("globalTargetRotation", Math.toDegrees(angle));
        debugAddData("targetRotation",  Math.toDegrees(clamped_target_rot));
        debugAddData("diff angle", diffAngle);
        debugAddData("currentPos", turretMotor.getCurrentPosition());
        debugAddData("current angle",Math.toDegrees(currentRotation));
        debugAddData("Tx", Tx);
        debugAddData("Error", Math.toDegrees(turretController.getE()));
        debugAddData("Output", turretController.currentOutput);


        debugAddData("isPointed",isPointed());
    }

    public LLResult limelightGetResult(int pipeline_index) {
        limelight.pipelineSwitch(pipeline_index);
        LLResult result = limelight.getLatestResult();
        while (result.getPipelineIndex() != pipeline_index) result = limelight.getLatestResult();
        return result;
    }

    public boolean limelightResultVaild(LLResult result) {
        return (result != null && result.isValid());
    }

    public int determinePattern() {
        if (limelightResultVaild(limelightGetResult(PGP))) {
            return PGP;
        }
        if (limelightResultVaild(limelightGetResult(GPP))) {
            return GPP;
        }
        if (limelightResultVaild(limelightGetResult(PPG))) {
            return PPG;
        }
        return -1;
    }

    public boolean atTargetPosition(){
        return Math.abs(turretController.getE()) <= Math.toRadians(1) && Math.abs(turretController.currentOutput) < 0.04;
    }

    public static double getAngle(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1,x2 - x1);
    }
    public static double getDiff(double angle1, double angle2){
        return ((angle1-angle2+Math.toRadians(180))%Math.toRadians(360)-Math.toRadians(180));
    }
    public boolean isPointed(){
        return mode == IDLE_MODE || (TagEx <= 1.0 && isInCamera);
    }
}