package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp(name="yawTest", group = "Tests")
public class yaw_turret_test extends OpMode {

    CRServo yaw;
    @Override
    public void init() {
        yaw = hardwareMap.get(CRServo.class, "yawservo");
        yaw.setPower(0);
    }

    @Override
    public void loop() {
        yaw.setPower(1);
    }
}
