package org.firstinspires.ftc.teamcode.Neel.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class StatusLight {
    public static String LIGHT_1_NAME = "light1";
    public static String LIGHT_2_NAME = "light2";
    public static double GREEN = 0.5;
    public static double BLUE = 0.611;

    private final List<Servo> lights = new ArrayList<>();
    private final StringBuilder found = new StringBuilder();
    private final StringBuilder missing = new StringBuilder();

    public StatusLight(HardwareMap hardwareMap) {
        addLight(hardwareMap, new String[]{LIGHT_1_NAME, "light 1", "Light1", "Light 1", "light_1", "light"});
        addLight(hardwareMap, new String[]{LIGHT_2_NAME, "light 2", "Light2", "Light 2", "light_2"});
    }

    private void addLight(HardwareMap hardwareMap, String[] names) {
        for (String name : names) {
            try {
                Servo light = hardwareMap.get(Servo.class, name);
                if (light != null) {
                    lights.add(light);
                    append(found, "\"" + name + "\"");
                    return;
                }
            } catch (Exception ignored) {
            }
        }
        append(missing, "\"" + names[0] + "\"");
    }

    private static void append(StringBuilder text, String part) {
        if (text.length() > 0) {
            text.append(", ");
        }
        text.append(part);
    }

    public void showAimed(boolean aimed) {
        setColor(aimed ? GREEN : BLUE);
    }

    public void setColor(double position) {
        for (Servo light : lights) {
            light.setPosition(position);
        }
    }

    public int count() {
        return lights.size();
    }

    public String status() {
        if (missing.length() == 0) {
            return "both lights found: " + found;
        }
        if (lights.isEmpty()) {
            return "NO lights found: add servos named " + missing + " to the robot config";
        }
        return "found " + found + ", NOT FOUND: " + missing + " (add it to the robot config)";
    }
}
