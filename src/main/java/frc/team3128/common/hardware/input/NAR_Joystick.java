package frc.team3128.common.hardware.input;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class NAR_Joystick {

    private Joystick stick;

    public POVButton povDown;
    public POVButton povLeft;
    public POVButton povRight;
    public POVButton povUp;

    private HashMap<Integer, JoystickButton> buttons;

    public NAR_Joystick(int deviceNumber) {
        buttons = new HashMap<Integer, JoystickButton>();
        stick = new Joystick(deviceNumber);

        for(int i = 1; i < 13; i++)
            buttons.put(i, new JoystickButton(stick, i));

        povDown = new POVButton(stick, 180);
        povLeft = new POVButton(stick, 270);
        povRight = new POVButton(stick, 90);
        povUp = new POVButton(stick, 0);

    }

    public JoystickButton getButton(int i) {
        return buttons.get(i);
    }

    public POVButton getUpPOVButton() {
        return povUp;
    }

    public POVButton getDownPOVButton() {
        return povDown;
    }

    public POVButton getLeftPOVButton() {
        return povLeft;
    }

    public POVButton getRightPOVButton() {
        return povRight;
    }

    public double getX() {
        return stick.getX();
    }

    public double getY() {
        return stick.getY();
    }

    public double getZ() {
        return stick.getZ();
    }

    public double getThrottle() {
        return stick.getThrottle();
    }

    public double getTwist() {
        return stick.getTwist();
    }

    
}
