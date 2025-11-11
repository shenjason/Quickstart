package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;


@Configurable
@TeleOp(name = "TeleOpMain", group = "TeleOp")
public class teleOpMain extends OpMode {

    public static boolean SIDE = Assembly.SIDE_BLUE;
    public static boolean DEBUG = true;

    Follower follower;

    Robot robot;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, telemetry, DEBUG, SIDE);
        robot.start();

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


        double speed = (gamepad1.left_bumper || gamepad1.right_bumper) ? 1 : 0.3d;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed,
                -gamepad1.right_stick_x * speed,
                true // Robot Centric
        );


        if (gamepad1.xWasPressed()){
            robot.fireBall();
        }

        if (gamepad1.bWasPressed()){
            robot.intakeMode();
        }

        if (gamepad1.leftBumperWasPressed()){
            robot.spinner.cycleSpinner();
        }

        robot.setIntakeState(gamepad1.a);

        telemetry.update();

    }
}
