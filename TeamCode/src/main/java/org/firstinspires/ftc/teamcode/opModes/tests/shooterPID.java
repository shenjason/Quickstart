package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Shooter;
import org.firstinspires.ftc.teamcode.util.ActionPress;
import org.firstinspires.ftc.teamcode.util.Assembly;


@Configurable
@TeleOp(name="ShooterPID", group = "Tests")
public class shooterPID extends OpMode {

    TelemetryManager telemetryManager;
    public static double P, I, D, TARGET_RPM;

    public static double yawP = 0.05d;

    public static double pitch = 60;
    Shooter s;



    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        s = new Shooter(hardwareMap, telemetry, true, Assembly.SIDE_BLUE);
        P = s.flywheelP; I=s.flyWheelI; D= s.flyWheelD;

    }

    @Override
    public void loop() {
        s.flywheelP = P; s.flyWheelD = D; s.flyWheelI = I;
        s.turret_yawP = yawP;
        s.setFlywheelRPM(TARGET_RPM);

        s.update();


        telemetryManager.addData("Error", s.flywheelPID.getE());
        telemetryManager.addData("Output", s.flywheelPID.currentOutput);
        telemetryManager.addData("TagSize", s.TagSize);


        telemetryManager.update(telemetry);


        telemetry.update();
    }
}
