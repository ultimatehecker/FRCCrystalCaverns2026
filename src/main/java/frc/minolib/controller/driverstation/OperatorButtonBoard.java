package frc.minolib.controller.driverstation;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorButtonBoard {
    private final GenericHID operatorHID;

    public OperatorButtonBoard(int port) {
        operatorHID = new GenericHID(port);
    }

    public boolean isConnected() {
        return operatorHID.isConnected();
    }

    public boolean getSPSTSwitch(int index) {
        if (index < 7 || index > 11) {
            throw new RuntimeException("Invalid driver override index " + Integer.toString(index) + ". Must be 7-11.");
        }

        return operatorHID.getRawButton(index + 1);
    }

    public boolean getMomentarySwitch(int index) {
        if (index < 0 || index > 5) {
            throw new RuntimeException("Invalid operator override index " + Integer.toString(index) + ". Must be 0-5.");
        }

        return operatorHID.getRawButton(index + 8);
    }

    public Trigger spstSwitch(int index) {
        return new Trigger(() -> getSPSTSwitch(index));
    }

    public Trigger momentarySwitch(int index) {
        return new Trigger(() -> getMomentarySwitch(index));
    }
}