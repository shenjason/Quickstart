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
    public static final boolean INTAKE_MODE = false, OUTTAKE_MODE = true;
    DcMotor spinnerMotor, intakeMotor;
    TouchSensor magSwitch;
    DistanceSensor distanceSensor;

    ActionPress resetPress, intakeCycleBallPress;

    public double cyclePos = 0;

    public double cyclePosOffset = 0;

    public boolean MODE = Spinner.INTAKE_MODE;

    boolean intaking = false;

    public boolean[] Occupied = {false, false, false};



    public Spinner(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        resetPress = new ActionPress(this::resetEncoder);
        intakeCycleBallPress = new ActionPress(this::intakeCycleBall);
    }

    void intakeCycleBall(){
        Occupied[(int)Math.floor(cyclePos)] = true;
        cycleSpinner();
    }

    public void resetSpinnerPos(){
        spinnerMovingMode();
        spinnerMove();
        while (!magSwitch.isPressed()) {}
        spinnerStop();
        spinnerCycleMode();
    }

    public boolean isBallAtPos(){

        int pos = (cyclePos > 2.5)? 1 : (int)Math.floor(cyclePos) + 1;

        if (pos > 2) pos = 0;


        return Occupied[pos];
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

    public void setIntakingState(boolean state){
        intaking = state;
    }

    public void cycleSpinner(){
        cyclePos ++;
        updateCyclePos();
    }

    public void updateCyclePos(){
        spinnerMotor.setTargetPosition((int)(3 * 141.1d * cyclePos) + ((cyclePos > 2.5)? 30 : 0));
    }

    public void intakeCycle(){
        if (MODE == Spinner.INTAKE_MODE) return;
        cyclePosOffset = 0;
        cyclePos = Math.floor(cyclePos);
        MODE = Spinner.INTAKE_MODE;
        updateCyclePos();
    }

    public void outtakeCycle(){
        if (MODE == Spinner.OUTTAKE_MODE) return;
        intakeMotor.setPower(0);
        cyclePosOffset = 0.5;
        cyclePos += cyclePosOffset;
        MODE = Spinner.OUTTAKE_MODE;
        updateCyclePos();
    }

    public boolean isBall(){
        return distanceSensor.getDistance(DistanceUnit.MM) < 30;
    }


    @Override
    public void hardwareInit() {
        spinnerMotor = hardwareMap.get(DcMotor.class, "spinnermotor");
        magSwitch = hardwareMap.get(TouchSensor.class, "Touch sensor");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "intakesensor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakemotor");
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
        debugAddData("isOccupied", isBallAtPos());

        if (MODE == Spinner.INTAKE_MODE){
            intakeMotor.setPower((intaking && !isBall())? 1 : 0);

            intakeCycleBallPress.update(isBall(), !Occupied[(cyclePos > 2.5)? 0 : (int)Math.floor(cyclePos)]);
        }
    }
}
