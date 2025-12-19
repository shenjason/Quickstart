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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;

@Autonomous(name = "Auto Blue (12 artifact non-overflow)", group = "Autonomous", preselectTeleOp = "TeleOpMain(Blue)")
@Configurable // Panels
public class autoBlue12C extends OpMode {

    public boolean SIDE = Assembly.SIDE_BLUE;
    public double ROT = Math.toRadians(180);
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    protected Paths paths; // Paths defined in the Paths class

    public Robot robot;
    public static double SPEED = 0.8;
    Timer timer;

    public void setSIDE(){}


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        setSIDE();

        robot = new Robot(hardwareMap, telemetry,follower,false, SIDE);

        follower.setMaxPower(SPEED);
        follower.setMaxPowerScaling(SPEED);

        paths = new Paths(follower, SIDE, ROT); // Build paths

        follower.setStartingPose(paths.x(paths.startPose));
        robot.intake(false);


        panelsTelemetry.debug("Status", "Initialized");

        pathState = 0;
        timer = new Timer();
        telemetry.addData("pos x",paths.start_shoot.endPoint().getX());
        telemetry.addData("pos y", paths.start_shoot.endPoint().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        robot.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//
//        panelsTelemetry.update();
//        telemetry.update();
    }

    public static class Paths {

        public boolean SIDE = Assembly.SIDE_BLUE;
        public Pose startPose = (new Pose(26.200, 130.000, Math.toRadians(52)));
        public Pose shoot = (new Pose(42,102, Math.toRadians(180)));
        public Pose ready1 = (new Pose(46.0,84.0,  Math.toRadians(180)));
        public Pose load1 = (new Pose(22.0,84.0, Math.toRadians(180)));

        public Pose gatePos = (new Pose(19, 72, Math.toRadians(180)));
        public Pose ready2 = (new Pose(46.0,60.0, Math.toRadians(180)));
        public Pose load2 = (new Pose(25.0, 60.0,  Math.toRadians(180)));
        public Pose ready3 = (new Pose(46.0,36.0, Math.toRadians(180)));
        public Pose load3 = (new Pose(23.0,36.0, Math.toRadians(180)));
        public Pose end = (new Pose(36.0,36.0, Math.toRadians(180)));

        public PathChain start_shoot, shoot_ready1, ready1_load1, load1_shoot, shoot_ready2,ready2_load2, load2_shoot, shoot_ready3, ready3_load3, load3_shoot, shoot_end, load1_gate, gate_shoot;

        public Paths(Follower follower, boolean SIDE, double ROT) {
            this.SIDE = SIDE;


            start_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(startPose), x(shoot))
                    )
                    .setLinearHeadingInterpolation(SIDE ? startPose.getHeading() : Math.toRadians(180)-startPose.getHeading(), ROT)
                    .build();

            shoot_ready1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot), x(ready1))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            ready1_load1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready1), x(load1))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            load1_gate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(x(load1), x(new Pose(32, 71)), x(gatePos))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            gate_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(gatePos), x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            load1_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load1), x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            shoot_ready2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot), x(ready2))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            ready2_load2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready2),x(load2))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            load2_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load2),x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            shoot_ready3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot),x(ready3))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            ready3_load3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready3),x(load3))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            load3_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load3),x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            shoot_end = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(x(shoot), x(end))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();
        }

        public Pose x(Pose p){
            return (SIDE) ? p : new Pose(144-p.getX(), p.getY(), p.getHeading() - Math.toRadians(180));
        }
    }

    public int autonomousPathUpdate() {
        switch(pathState){
            case 0:
                //set turret angle to 45 degrees
                robot.shooter.setFlywheelRPM(2550);
                robot.shooter.turret.debugTargetAngle = Math.toRadians(37);
                follower.followPath(paths.start_shoot, true);
                pathState++;
                break;
            case 1:
            case 6:
            case 10:
            case 14:
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
                    follower.setMaxPower(0.45);
                    follower.followPath(paths.ready1_load1,true);
                    pathState++;
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    robot.intake(false);
                    follower.setMaxPower(SPEED);
                    follower.followPath(paths.load1_gate,true);
                    pathState++;
                }
                break;
            case 5:{
                if (!follower.isBusy()){
                    follower.followPath(paths.gate_shoot,true);
                    pathState++;
                }
                break;
            }
            case 7:
                if(!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready2,true);
                    pathState++;
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    robot.intake(true);
                    follower.setMaxPower(0.45);
                    follower.followPath(paths.ready2_load2,true);
                    pathState++;
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    robot.intake(false);
                    follower.setMaxPower(SPEED);
                    follower.followPath(paths.load2_shoot,true);
                    pathState++;
                }
                break;
            case 11:
                if(!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready3,true);
                    pathState++;
                }
                break;
            case 12:

                if(!follower.isBusy()){
                    robot.intake(true);
                    follower.setMaxPower(0.45);
                    follower.followPath(paths.ready3_load3,true);
                    pathState++;
                }
                break;
            case 13:
                if(!follower.isBusy()){
                    robot.intake(false);
                    follower.setMaxPower(SPEED);
                    follower.followPath(paths.load3_shoot,true);
                    pathState++;
                }
                break;
            case 15:
                if(!robot.shooter.shooting){
                    robot.shooter.offShooter();
                    follower.followPath(paths.shoot_end,true);
                    pathState++;
                    timer.resetTimer();
                }
                break;
            case 16:
                if (timer.getElapsedTimeSeconds() > 0.5){
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
