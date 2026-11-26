package org.firstinspires.ftc.teamcode.Neel.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
@Disabled
@Autonomous(name = "AutoBlue", group = "Glitch")
public class Autoblue extends CompAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
