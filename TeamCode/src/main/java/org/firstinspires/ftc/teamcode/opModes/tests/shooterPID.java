package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Shooter;
import org.firstinspires.ftc.teamcode.assemblies.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.ActionPress;
import org.firstinspires.ftc.teamcode.util.Assembly;


@Configurable
@TeleOp(name="ShooterPID", group = "Tests")
public class shooterPID extends OpMode {

    TelemetryManager telemetryManager;
    Follower follower;
    public static double P, I, D, TARGET_RPM;
    public static boolean LOAD = true;


    Shooter s;



    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        s = new Shooter(hardwareMap, telemetry, follower, true, Assembly.SIDE_BLUE);
        if (LOAD){
            P = s.flywheelP;I=s.flyWheelI; D= s.flyWheelD;
        }

        s.turret.mode = Turret.IDLE_MODE;
    }

    @Override
    public void loop() {

        s.flywheelP = P; s.flyWheelD = D; s.flyWheelI = I;
        s.setFlywheelRPM(TARGET_RPM);

        if (gamepad1.aWasPressed()){
            s.Shoot();
        }


        s.update();


        telemetryManager.addData("Error", s.flywheelPID.getE());
        telemetryManager.addData("Output", s.flywheelPID.currentOutput);
        telemetryManager.addData("TagSize", s.TagSize);


        telemetryManager.update(telemetry);


        telemetry.update();
    }
}
