package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ActionPress;
import org.firstinspires.ftc.teamcode.util.Assembly;



public class Spinner extends Assembly {

    DcMotor spinnerMotor;
    DistanceSensor distanceSensor;

    ActionPress resetPress;



    public Spinner(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        resetPress = new ActionPress(this::resetEncoder);
    }

    public void spinnerMove(){
        spinnerMotor.setPower(0.3);
    }

    public void spinnerReverseMove(){ spinnerMotor.setPower(-0.3); }
    public void spinnerStop() { spinnerMotor.setPower(0);}

    public void spinnerMovingMode(){
        spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinnerMotor.setPower(0);
    }

    public void spinnerCycleMode(){
        spinnerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinnerMotor.setTargetPosition(0);
        spinnerMotor.setPower(0.8);
        spinnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void cycleSpinner(){
        spinnerMotor.setTargetPosition((int)(3 * 141.1d + 30));
    }


    @Override
    public void hardwareInit() {
        spinnerMotor = hardwareMap.get(DcMotor.class, "spinnermotor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "intakesensor");

        spinnerCycleMode();
    }

    void resetEncoder(){
        spinnerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinnerMotor.setTargetPosition(0);
        spinnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void update() {

        debugAddData("Distance", distanceSensor.getDistance(DistanceUnit.MM));
        resetPress.update(distanceSensor.getDistance(DistanceUnit.MM) > 80 && distanceSensor.getDistance(DistanceUnit.MM) < 100);
    }
}
