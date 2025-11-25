package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Assembly;



public class Turret extends Assembly
{
    private Limelight3A limelight;
    private IMU imu;

    private DcMotor turretMotor;
    private Follower follower;

    public final static int GPP = 0, PGP = 1, PPG = 2, BLUE_TARGET_LINE = 3, RED_TARGET_LINE = 4, TRACKING_MODE = 0, IDLE_MODE = 1;
    public int mode = IDLE_MODE;


    public Turret(HardwareMap _hardwareMap, Telemetry _t, Follower f, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        follower = f;
    }

    @Override
    public void hardwareInit() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        turretMotor =  hardwareMap.get(DcMotor.class,"turret");




    }



    @Override
    public void update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (limelightResultVaild(llResult)) {
            turretMotor.setPower(0);
            Pose3D botPose = llResult.getBotpose();

            debugAddData("Tx", llResult.getTx());
            debugAddData("Ty", llResult.getTy());
            debugAddData("Ta", llResult.getTa());
        }
        else{
            getAngle(follower.getPose().getX(),follower.getPose().getY(),72,0)+follower.getPose().getHeading()



        }

        debugAddData("PoseX",  follower.getPose().getX());
        debugAddData("PoseY",  follower.getPose().getY());
        debugAddData("Heading",  Math.toDegrees(follower.getPose().getHeading()));

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
    public double getAngle(double x, double y, double tx, double ty){
        return Math.atan2(ty-y, tx-x);
    }
    public 
}
