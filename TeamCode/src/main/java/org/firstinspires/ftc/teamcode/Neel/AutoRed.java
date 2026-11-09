package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;

@Autonomous(name = "AutoRed", group = "Glitch")
public class AutoRed extends CompAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
