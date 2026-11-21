package org.firstinspires.ftc.teamcode.Neel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;

@Autonomous(name = "Turret Auto Red", group = "Glitch", preselectTeleOp = "Odometry Turret Shooting")
public class TurretAutoRed extends TurretAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
