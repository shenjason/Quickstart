package org.firstinspires.ftc.teamcode.opModes.tests;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;

@Configurable
@TeleOp(name="ShooterPID", group = "Tests")
public class turretTest extends OpMode {
    TelemetryManager telemetryManager;

    Turret t;

    Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        t = new Turret(hardwareMap, telemetry, follower, true, Assembly.SIDE_BLUE);
    }

    @Override
    public void loop() {
        follower.update();
        t.update();
    }
}
