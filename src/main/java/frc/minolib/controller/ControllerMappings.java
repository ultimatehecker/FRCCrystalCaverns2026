package frc.minolib.controller;

import java.util.HashMap;
import java.util.Map;

public class ControllerMappings {
    public static final ControllerMapping XBOX_MAPPING;
    public static final ControllerMapping DUALSENSE_MAPPING;
    public static final ControllerMapping VEX_MAPPING;

    static {
        Map<String, Integer> xboxButtons = new HashMap<>();
        xboxButtons.put("A", 1);
        xboxButtons.put("B", 2);
        xboxButtons.put("X", 3);
        xboxButtons.put("Y", 4);
        xboxButtons.put("LeftBumper", 5);
        xboxButtons.put("RightBumper", 6);
        xboxButtons.put("Back", 7);
        xboxButtons.put("Start", 8);
        xboxButtons.put("LeftStick", 9);
        xboxButtons.put("RightStick", 10);

        Map<String, Integer> xboxAxes = new HashMap<>();
        xboxAxes.put("LeftX", 0);
        xboxAxes.put("LeftY", 1);
        xboxAxes.put("RightX", 2);
        xboxAxes.put("RightY",3);
        xboxAxes.put("RightTrigger", 5);
        xboxAxes.put("LeftTrigger", 4);

        XBOX_MAPPING = new ControllerMapping(xboxButtons, xboxAxes);

        Map<String, Integer> dualSenseButtons = new HashMap<>();
        dualSenseButtons.put("A", 1);
        dualSenseButtons.put("B", 2);
        dualSenseButtons.put("X", 3);
        dualSenseButtons.put("Y", 4);
        dualSenseButtons.put("LeftBumper", 5);
        dualSenseButtons.put("RightBumper", 6);
        dualSenseButtons.put("Back", 7);
        dualSenseButtons.put("Start", 8);
        dualSenseButtons.put("LeftStick", 10);
        dualSenseButtons.put("RightStick", 11);

        Map<String, Integer> dualSenseAxes = new HashMap<>();
        dualSenseAxes.put("LeftX", 0);
        dualSenseAxes.put("LeftY", 1);
        dualSenseAxes.put("RightX", 4);
        dualSenseAxes.put("RightY", 5);
        dualSenseAxes.put("LeftTrigger", 2);
        dualSenseAxes.put("RightTrigger", 3);

        DUALSENSE_MAPPING = new ControllerMapping(dualSenseButtons, dualSenseAxes);

        Map<String, Integer> vexButtons = new HashMap<>();
        vexButtons.put("A", 2);
        vexButtons.put("B", 1);
        vexButtons.put("X",4);
        vexButtons.put("Y", 3);
        vexButtons.put("LeftBumper1", 5);
        vexButtons.put("RightBumper1", 6);
        vexButtons.put("LeftBumper2", 7);
        vexButtons.put("RightBumper2", 8);
        vexButtons.put("Start", 9);

        Map<String, Integer> vexAxes = new HashMap<>();
        vexAxes.put("LeftX", 0);
        vexAxes.put("LeftY", 1);
        vexAxes.put("RightX", 2);
        vexAxes.put("RightY", 3);

        VEX_MAPPING = new ControllerMapping(vexButtons, vexAxes);
    }
}