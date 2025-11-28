package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
    public double P=1,I=0,D=0;
    public PIDcontroller turretController;
    public boolean isInCamera;
    private double targetRotation;
    public double Ta;
    private double Tx;
    private Limelight3A limelight;

    private DcMotor turretMotor;
    private Follower follower;

    private double targetPointX;

    private static double  TARGETBLUEX=11, TARGETY=137.1, TARGETREDX=144-TARGETBLUEX;

    public final static int GPP = 0, PGP = 1, PPG = 2, BLUE_TARGET_LINE = 3, RED_TARGET_LINE = 4, TRACKING_MODE = 0, IDLE_MODE = 1;
    public int mode = IDLE_MODE;
    public double offsetAngle = 0;

    public double debugTargetAngle = 0;


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

        limelight.pipelineSwitch((side) ? BLUE_TARGET_LINE : RED_TARGET_LINE);
        targetPointX = (side) ? TARGETBLUEX : TARGETREDX;


        turretController = new PIDcontroller(P, I, D, -1, 1);

    }


    @Override
    public void update() {

        LLResult llResult = limelight.getLatestResult();
        Pose cp = follower.getPose();
        double angle = getAngle(cp.getX(),cp.getY(),targetPointX,TARGETY);
        Ta = 0; Tx = 0;
        double robotAngle = cp.getHeading();
        isInCamera = limelightResultVaild(llResult);
        if (isInCamera) {
            Ta = llResult.getTx();
            Tx = llResult.getTx();
        }

        double diffAngle = (angle - robotAngle + Math.PI) % (2*Math.PI) - Math.PI;
        targetRotation = diffAngle - Math.toRadians(Tx);
        double currentRotation = (turretMotor.getCurrentPosition()/145.1d*0.16d*2d*Math.PI) - offsetAngle;

        double clamped_target_rot = targetRotation;
        if (Math.abs(targetRotation)>=Math.PI/2) clamped_target_rot = Math.PI/2*Math.signum(targetRotation);

        if (debug) turretController.p = P; turretController.i = I; turretController.d = D;

        turretMotor.setPower(turretController.step((mode == Turret.TRACKING_MODE) ? clamped_target_rot : debugTargetAngle, currentRotation));

        debugAddLine("Turret");
        debugAddData("PoseX", cp.getX());
        debugAddData("PoseY", cp.getY());
        debugAddData("Heading", Math.toDegrees(cp.getHeading()));
        debugAddData("globalTargetRotation", Math.toDegrees(angle));
        debugAddData("targetRotation",  Math.toDegrees(clamped_target_rot));
//        debugAddData("Angle bot->tag approx",Math.toDegrees(diffAngle));

        debugAddData("currentRotation", Math.toDegrees(currentRotation));
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

    public static double getAngle(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1,x2 - x1);
    }
    public boolean isPointed(){
        return mode == IDLE_MODE || (Math.abs(Tx) <= 5.0 && isInCamera);
    }
}