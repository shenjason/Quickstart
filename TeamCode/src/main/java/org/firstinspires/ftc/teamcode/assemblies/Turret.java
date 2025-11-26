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



public class Turret extends Assembly {
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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        limelight.setPollRateHz(30);

        limelight.pipelineSwitch((side) ? BLUE_TARGET_LINE : RED_TARGET_LINE);
    }


    @Override
    public void update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llResult = limelight.getLatestResult();
        Pose cp = follower.getPose();
        double ANGLE = getAngle(cp.getX(),cp.getY(),72.0,0.0);
        if (limelightResultVaild(llResult)) {
            //Pose3D botPose = llResult.getBotpose();

            debugAddData("Tx", llResult.getTx());
            debugAddData("Ty", llResult.getTy());
            debugAddData("Ta", llResult.getTa());
//            if (Math.abs(llResult.getTx())<5.0)
//            {
//                turretMotor.setPower(Math.signum(ANGLE));
//            }
        }
//        else{
//            if (ANGLE<0.0){
//                turretMotor.setPower(0.1);
//            }
//            else{
//                turretMotor.setPower(-0.1);
//            }
//        }

        debugAddData("PoseX", cp.getX());
        debugAddData("PoseY", cp.getY());
        debugAddData("Heading", Math.toDegrees(cp.getHeading()));
        debugAddData("Angle bot->tag approx", ANGLE);

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

    public static double getAngle(double a, double b, double a2, double b2) {
        double sign = Math.signum(a2 - a);

        double atanRad = Math.atan2(b2 - b,a2 - a);

        double term1 = Math.toRadians(90) - atanRad;
        double term2 = -Math.toRadians(180) * ((1 - sign) / 2.0);
        double term3 = Math.toRadians(90) * (1 - Math.abs(sign));

        return term1 + term2 + term3;
    }
}
