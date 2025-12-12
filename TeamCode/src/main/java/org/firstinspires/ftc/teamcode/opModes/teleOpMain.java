package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;


@Configurable
@TeleOp(name = "TeleOpMain(Blue)", group = "TeleOp")
public class teleOpMain extends OpMode {

    public static boolean SIDE = Assembly.SIDE_BLUE;
    public static boolean DEBUG = true;

    Follower follower;

    Robot robot;

    IMU imu;
    public void setSIDE() {};

    @Override
    public void init() {
        setSIDE();
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, follower, DEBUG, SIDE);

        follower.setStartingPose(new Pose((SIDE) ? 36 : 108, 36, (SIDE) ? Math.toRadians(180) : 0));

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        imu.resetYaw();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.update();
        robot.update();


        double speed = (gamepad1.right_bumper) ? 1 : 0.5d;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed,
                -gamepad1.right_stick_x * speed * 0.5,
                true // Robot Centric
        );



        robot.intake(gamepad1.left_bumper);

        if (gamepad1.yWasPressed()){
            if (robot.shooter.turret.mode == Turret.IDLE_MODE){
                robot.tracking();
            }else{
                robot.idle();
            }
        }

        if (gamepad1.bWasPressed() && !robot.shooter.shooting){
            robot.shoot();
        }

        if (gamepad1.dpadDownWasPressed()){
            follower.setPose(new Pose(72, 72, -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + ((SIDE) ? Math.toRadians(0) : Math.toRadians(180))));
        }

        if (gamepad1.dpadRightWasPressed()){
            robot.shooter.turret.offsetAngle += Math.toRadians(5);
        }
        if (gamepad1.dpadLeftWasPressed()){
            robot.shooter.turret.offsetAngle -= Math.toRadians(5);
        }


        telemetry.update();

    }
}
