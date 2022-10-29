package frc.team3128.common.hardware.input;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NAR_XboxController extends XboxController {

    private String buttonNames[] = {
        "A",
        "B",
        "X",
        "Y",
        "LeftBumper",
        "RightBumper",
        "Back",
        "Start",
        "LeftStick",
        "RightStick"
    };
    
    private HashMap<String, Trigger> buttons;
    private Trigger[] povButtons;

    public NAR_XboxController(int port) {
        super(port);
        buttons = new HashMap<String, Trigger>();
        povButtons = new Trigger[8];
        for (int i = 0; i < 10; i++) {
            int n = i + 1;
            buttons.put(buttonNames[i], new Trigger (() -> getRawButton(n)));
        }   
        for (int i = 0; i < 8; i++) {
            int n = i;
            povButtons[i] = new Trigger (() -> getPOV() == n * 45);
        }
    }

    public Trigger getButton(String buttonName) {
        return buttons.get(buttonName);
    }

    public Trigger getLeftTrigger() {
        return new Trigger (() -> getLeftTriggerAxis() >= 0.5);
    }

    public Trigger getRightTrigger() {
        return new Trigger (() -> getRightTriggerAxis() >= 0.5);
    }

    public Trigger getPOVButton(int i) {
        return povButtons[i];
    }

    public Trigger getUpPOVButton() {
        return getPOVButton(0);
    }

    public Trigger getDownPOVButton() {
        return getPOVButton(4);
    }

    public Trigger getLeftPOVButton() {
        return getPOVButton(6);
    }

    public Trigger getRightPOVButton() {
        return getPOVButton(2);
    }
}