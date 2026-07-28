package frc.minolib.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandSimulatedXboxController extends CommandXboxController {
    private final SimulatedXboxController m_hid;
    private final ControllerMapping mapping;

    public CommandSimulatedXboxController(int port) {
        super(port);
        switch (ControllerConstants.kSimulationControllerType) {
            case XBOX:
                mapping = ControllerMappings.XBOX_MAPPING;
                m_hid = new SimulatedXboxController(port, mapping);
                break;
            case DUAL_SENSE:
                mapping = ControllerMappings.DUALSENSE_MAPPING;
                m_hid = new SimulatedXboxController(port, mapping);
                break;
            case VEX:
                mapping = ControllerMappings.VEX_MAPPING;
                m_hid = new SimulatedXboxController(port, mapping);
                break;
            default:
                mapping = ControllerMappings.XBOX_MAPPING;
                m_hid = new SimulatedXboxController(port, mapping);
                break;
        }
    }

    @Override
    public XboxController getHID() {
        return m_hid;
    }

    @Override
    public Trigger a(EventLoop loop) {
        return button(mapping.getButton("A"), loop);
    }

    @Override
    public Trigger b(EventLoop loop) {
        return button(mapping.getButton("B"), loop);
    }

    @Override
    public Trigger x(EventLoop loop) {
        return button(mapping.getButton("X"), loop);
    }

    @Override
    public Trigger y(EventLoop loop) {
        return button(mapping.getButton("Y"), loop);
    }

    @Override
    public Trigger leftBumper(EventLoop loop) {
        return button(mapping.getButton("LeftBumper"), loop);
    }

    @Override
    public Trigger rightBumper(EventLoop loop) {
        return button(mapping.getButton("RightBumper"), loop);
    }

    @Override
    public Trigger back(EventLoop loop) {
        return button(mapping.getButton("Back"), loop);
    }

    @Override
    public Trigger start(EventLoop loop) {
        return button(mapping.getButton("Start"), loop);
    }

    @Override
    public Trigger leftStick(EventLoop loop) {
        return button(mapping.getButton("LeftStick"), loop);
    }

    @Override
    public Trigger rightStick(EventLoop loop) {
        return button(mapping.getButton("RightStick"), loop);
    }

    @Override
    public Trigger leftTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(mapping.getAxis("LeftTrigger"), threshold, loop);
    }

    @Override
    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(mapping.getAxis("RightTrigger"), threshold, loop);
    }

    @Override
    public double getLeftX() {
        return getRawAxis(mapping.getAxis("LeftX"));
    }

    @Override
    public double getRightX() {
        return getRawAxis(mapping.getAxis("RightX"));
    }

    @Override
    public double getLeftY() {
        return getRawAxis(mapping.getAxis("LeftY"));
    }

    @Override
    public double getRightY() {
        return getRawAxis(mapping.getAxis("RightY"));
    }

    @Override
    public double getLeftTriggerAxis() {
        return m_hid.getLeftTriggerAxis();
    }

    @Override
    public double getRightTriggerAxis() {
        return m_hid.getRightTriggerAxis();
    }
}