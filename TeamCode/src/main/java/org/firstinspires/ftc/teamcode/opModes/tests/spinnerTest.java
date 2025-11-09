package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Spinner;
import org.firstinspires.ftc.teamcode.util.Assembly;

@Configurable
@TeleOp(name="SpinnerTest", group = "Tests")
public class spinnerTest extends OpMode {
    Spinner spinner;
    @Override
    public void init() {
        spinner = new Spinner(hardwareMap, telemetry, true, Assembly.SIDE_BLUE);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()){
            spinner.cycleSpinner();
        }
        spinner.update();
        telemetry.update();


        if (gamepad1.bWasPressed()){
            spinner.intakeCycle();
        }
        if (gamepad1.yWasPressed()){
            spinner.outtakeCycle();
        }
    }
}
