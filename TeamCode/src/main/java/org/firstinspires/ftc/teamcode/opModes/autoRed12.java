package org.firstinspires.ftc.teamcode.opModes;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Assembly;

@Autonomous(name = "Auto Blue (12 artifact)", group = "Autonomous", preselectTeleOp = "TeleOpMain(Red)")
@Configurable // Panels
public class autoRed12 extends autoBlue12{
    @Override
    public void setSIDE(){
        SIDE = Assembly.SIDE_RED;
    }
}
