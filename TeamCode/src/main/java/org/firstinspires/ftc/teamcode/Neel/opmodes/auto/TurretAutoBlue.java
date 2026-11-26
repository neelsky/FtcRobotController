package org.firstinspires.ftc.teamcode.Neel.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Neel.subsystems.Alliance;

@Autonomous(name = "Turret Auto Blue", group = "Glitch", preselectTeleOp = "Odometry Turret Shooting")
public class TurretAutoBlue extends TurretAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
