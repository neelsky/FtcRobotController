package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
@Disabled
@Autonomous(name = "AutoRed", group = "Glitch")
public class AutoRed extends CompAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
