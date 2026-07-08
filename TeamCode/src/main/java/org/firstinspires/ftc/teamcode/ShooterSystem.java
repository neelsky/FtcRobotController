package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterSystem implements Subsystem {
    // put hardware, commands, etc here

    @Override
    public void initialize() {
        MotorEx Shooter = new MotorEx("Shooter");
        // initialization logic (runs on init)
    }

}
