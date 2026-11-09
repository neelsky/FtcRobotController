package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;

@Autonomous(name = "AutoBlue", group = "Glitch")
public class Autoblue extends CompAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
