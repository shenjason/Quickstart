package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.Sequencer;

import java.util.List;

public class Robot extends Assembly {

    public Spinner spinner;
    public Shooter shooter;


    Sequencer fireballSequence;

    public Robot(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        shooter = new Shooter(_hardwareMap, _t, _debug, _side);
        spinner = new Spinner(_hardwareMap, _t, _debug, _side);

        fireballSequence = new Sequencer(List.of(
                () -> spinner.outtakeCycle(),
                () -> shooter.autoAdjustShooterParameters(),
                () -> shooter.shooterSequence.start(),
                () -> spinner.cycleSpinner()
        ), List.of(
                0d, 0d, 500d, 1d
        ), List.of(
                Sequencer.defaultCondition(),
                Sequencer.defaultCondition(),
                () -> shooter.atTargetFlywheelRPM(),
                Sequencer.defaultCondition()
        ));
    }

    public void start(){
        spinner.resetSpinnerPos();
    }

    @Override
    public void hardwareInit() { }


    public void fireBall(){
        fireballSequence.start();
    }

    public void intakeMode(){
        spinner.intakeCycle();
        shooter.setFlywheelRPM(0);
    }

    public void setIntakeState(boolean state){
        spinner.setIntakingState(state);
    }

    public void setIntakeRejectState(boolean state){
        spinner.rejectMode(state);
    }

    @Override
    public void update() {

        spinner.update();
        shooter.update();


        fireballSequence.update();
    }
}
