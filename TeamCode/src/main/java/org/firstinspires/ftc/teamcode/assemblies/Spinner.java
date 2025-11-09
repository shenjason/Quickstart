package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ActionPress;
import org.firstinspires.ftc.teamcode.util.Assembly;



public class Spinner extends Assembly {

    DcMotor spinnerMotor;
    TouchSensor magSwitch;

    ActionPress resetPress;

    public double cyclePos = 0;

    public double cyclePosOffset = 0;



    public Spinner(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        resetPress = new ActionPress(this::resetEncoder);
    }
    public void spinnerMove(){
        spinnerMotor.setPower(0.5);
    }

    public void spinnerReverseMove(){ spinnerMotor.setPower(-0.5); }
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
        cyclePos ++;
        updateCyclePos();
    }

    public void updateCyclePos(){
        spinnerMotor.setTargetPosition((int)(3 * 141.1d * cyclePos) + ((cyclePos > 2.5)? 30 : 0));
    }

    public void intakeCycle(){
        cyclePosOffset = 0;
        cyclePos = Math.floor(cyclePos);
        updateCyclePos();
    }

    public void outtakeCycle(){
        cyclePosOffset = 0.5;
        cyclePos += cyclePosOffset;
        updateCyclePos();
    }


    @Override
    public void hardwareInit() {
        spinnerMotor = hardwareMap.get(DcMotor.class, "spinnermotor");
        magSwitch = hardwareMap.get(TouchSensor.class, "Touch sensor");

        spinnerCycleMode();
    }

    void resetEncoder(){
        spinnerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinnerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cyclePos = cyclePosOffset;
        updateCyclePos();
    }

    @Override
    public void update() {
        resetPress.update(magSwitch.isPressed());
        debugAddData("CyclePos", cyclePos);
        debugAddData("magSwitchState", magSwitch.isPressed());
        debugAddData("CyclePosOffset", cyclePosOffset);
    }
}
