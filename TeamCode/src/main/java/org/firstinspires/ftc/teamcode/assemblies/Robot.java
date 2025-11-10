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

        shooter = new Shooter(_hardwareMap, _t, _debug, _side);
        spinner = new Spinner(_hardwareMap, _t, _debug, _side);

        fireballAllSequence = new Sequencer(List.of(
                () -> spinner.outtakeCycle(),
                () -> shooter.autoAdjustShooterParameters(),
                () -> spinner.cycleSpinner(),
                () -> shooter.shooterSequence.start(),
                () -> spinner.cycleSpinner(),
                () -> shooter.shooterSequence.start(),
                () -> spinner.cycleSpinner(),
                () -> shooter.shooterSequence.start(),
                () -> shooter.offShooter()
        ), List.of(
                0d, 0d, 500d, 0.5d, 0d, 0.5d, 0d, 0.5d, 1d
        ), List.of(
                Sequencer.defaultCondition(),
                Sequencer.defaultCondition(),
                () -> shooter.atTargetFlywheelRPM(),
                () -> shooter.isBall(),
                Sequencer.defaultCondition(),
                () -> shooter.isBall(),
                Sequencer.defaultCondition(),
                () -> shooter.isBall(),
                Sequencer.defaultCondition()
        ));
    }

    public void start(){
        spinner.resetSpinnerPos();
    }

    @Override
    public void hardwareInit() { }


    public void fireAllBalls(){
        fireballAllSequence.start();
    }

    public void intakeMode(){
        spinner.intakeCycle();
    }

    public void setIntakeState(boolean state){
        spinner.setIntakingState(state);
    }

    @Override
    public void update() {

        spinner.update();
        shooter.update();


        fireballAllSequence.update();
    }
}
