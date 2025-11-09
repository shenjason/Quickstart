package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.Sequencer;

import java.util.List;

public class Robot extends Assembly {

    Spinner spinner;
    Shooter shooter;


    Sequencer fireballAllSequence;

    public Robot(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
    }

    @Override
    public void hardwareInit() {
        shooter = new Shooter(hardwareMap, t, debug, side);
        spinner = new Spinner(hardwareMap, t, debug, side);


        fireballAllSequence = new Sequencer(List.of(
                () -> spinner.outtakeCycle(),
                () -> shooter.autoAdjustShooterParameters(),
                () -> spinner.cycleSpinner(),
                () -> shooter.shooterSequence.start(),
                () -> spinner.cycleSpinner(),
                () -> shooter.shooterSequence.start(),
                () -> spinner.cycleSpinner(),
                () -> shooter.shooterSequence.start()
        ), List.of(
                0d, 0d, 5d, 0.5d, 0d, 0.5d, 0d, 0.5d
        ), List.of(
                Sequencer.defaultCondition(),
                Sequencer.defaultCondition(),
                () -> shooter.atTargetFlywheelRPM(),
                () -> shooter.isBall(),
                Sequencer.defaultCondition(),
                () -> shooter.isBall(),
                Sequencer.defaultCondition(),
                () -> shooter.isBall()
        ));
    }


    public void fireAllBalls(){
        fireballAllSequence.start();
    }

    @Override
    public void update() {
        debugAddLine("Shooter: ");
        shooter.update();
        debugAddLine("Spinner: ");
        spinner.update();


        fireballAllSequence.update();
    }
}
