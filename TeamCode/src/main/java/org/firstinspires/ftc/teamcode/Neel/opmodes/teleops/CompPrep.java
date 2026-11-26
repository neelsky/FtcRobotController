package org.firstinspires.ftc.teamcode.Neel.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;
@Disabled
@TeleOp(name = "Comp Blue", group = "Glitch")
public class CompPrep extends CompTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
