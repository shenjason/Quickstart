package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;

@Autonomous(name = "Auto Blue", group = "Autonomous", preselectTeleOp = "TeleOpMain(Blue)")
@Configurable // Panels
public class autoblue extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    public Robot robot;

    public double SPEED = 0.8;
    Timer timer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        robot = new Robot(hardwareMap, telemetry,follower,true, Assembly.SIDE_BLUE);

        follower.setMaxPower(SPEED);
        follower.setMaxPowerScaling(SPEED);
        paths = new Paths(follower); // Build paths

        follower.setStartingPose(paths.startPose);
        robot.intake(false);


        panelsTelemetry.debug("Status", "Initialized");

        pathState = 0;
        timer = new Timer();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        robot.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public Pose startPose = new Pose(26.200, 130.000, Math.toRadians(52));
        public Pose shoot = new Pose(42,102, Math.toRadians(180));
        public Pose ready1 = new Pose(44.0,84.0,  Math.toRadians(180));
        public Pose load1 = new Pose(24.0,84.0, Math.toRadians(180));
        public Pose ready2 = new Pose(44.0,60.0, Math.toRadians(180));
        public Pose load2 = new Pose(24.0, 60.0,  Math.toRadians(180));
        public Pose ready3 = new Pose(44.0,38.0, Math.toRadians(180));
        public Pose load3 = new Pose(24.0,38.0, Math.toRadians(180));
        public Pose end = new Pose(36.0,12.0, Math.toRadians(180));

        public PathChain start_shoot, shoot_ready1, ready1_load1, load1_shoot, shoot_ready2,ready2_load2, load2_shoot, shoot_ready3, ready3_load3, load3_shoot, shoot_end;

        public Paths(Follower follower) {
            start_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startPose, shoot)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(52), Math.toRadians(180))
                    .build();

            shoot_ready1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shoot, ready1)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ready1_load1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(ready1, load1)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            load1_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(load1, shoot)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot_ready2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shoot, ready2)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ready2_load2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(ready2,load2)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            load2_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(load2,shoot)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot_ready3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shoot,ready3)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            ready3_load3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(ready3,load3)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            load3_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(load3,shoot)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot_end = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(shoot, end)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch(pathState){
            case 0:
                //set turret angle to 45 degrees
                robot.shooter.setFlywheelRPM(2800);
                robot.shooter.turret.debugTargetAngle = Math.toRadians(40);
                follower.followPath(paths.start_shoot, true);
                pathState++;
                break;
            case 1:
            case 5:
            case 9:
            case 13:
                if (!follower.isBusy() && robot.shooter.canShoot()){
                    robot.shoot();
                    pathState++;
                }
                break;
            case 2:
                if (!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready1,true);
                    pathState++;
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    robot.intake(true);
                    follower.setMaxPower(0.4);
                    follower.followPath(paths.ready1_load1,true);
                    pathState++;
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    robot.intake(false);
                    follower.setMaxPower(SPEED);
                    follower.followPath(paths.load1_shoot,true);
                    pathState++;
                }
                break;
            case 6:
                if(!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready2,true);
                    pathState++;
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    robot.intake(true);
                    follower.setMaxPower(0.4);
                    follower.followPath(paths.ready2_load2,true);
                    pathState++;
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    robot.intake(false);
                    follower.setMaxPower(SPEED);
                    follower.followPath(paths.load2_shoot,true);
                    pathState++;
                }
                break;
            case 10:
                if(!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready3,true);
                    pathState++;
                }
                break;
            case 11:

                if(!follower.isBusy()){
                    robot.intake(true);
                    follower.setMaxPower(0.4);
                    follower.followPath(paths.ready3_load3,true);
                    pathState++;
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    robot.intake(false);
                    follower.setMaxPower(SPEED);
                    follower.followPath(paths.load3_shoot,true);
                    pathState++;
                }
                break;
            case 14:
                if(!robot.shooter.shooting){
                    robot.shooter.offShooter();
                    follower.followPath(paths.shoot_end,true);
                    pathState++;
                    timer.resetTimer();
                }
                break;
            case 15:
                if (timer.getElapsedTimeSeconds() > 1){
                    robot.shooter.turret.debugTargetAngle = 0;
                    pathState = -1;
                }
                break;
        }
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        return pathState;
    }
}
