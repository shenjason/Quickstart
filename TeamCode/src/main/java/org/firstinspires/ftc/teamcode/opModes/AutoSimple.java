package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "auto")
public class AutoSimple extends OpMode {

    DcMotor br, fr, bl, fl;

    @Override
    public void init() {
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");


        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start() {
        fr.setPower(0.5);
        br.setPower(0.5);
        fl.setPower(0.5);
        bl.setPower(0.5);

        try {
            wait(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
    }

    @Override
    public void loop() {

    }
}
