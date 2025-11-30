package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    public void setSIDE() {};

    @Override
    public void init() {
        setSIDE();
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, follower, DEBUG, SIDE);

        follower.setStartingPose(new Pose(75, 75));
        follower.updateDrivetrain();


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


        if (gamepad1.leftBumperWasPressed()){
            robot.idle();
        }

        robot.intake(gamepad1.left_bumper);

        if (gamepad1.yWasPressed()){
            if (robot.shooter.turret.mode == Turret.IDLE_MODE){
                robot.tracking();
            }else{
                robot.idle();
            }
        }

        if (gamepad1.bWasPressed()){
            robot.shoot();
        }


        telemetry.update();

    }
}
