package frc.team3128.hardware;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AN_Joystick {

    private Joystick stick;

    private JoystickButton trigger;

    private ArrayList<JoystickButton> buttons;

    public AN_Joystick(int deviceNumber){
        stick = new Joystick(deviceNumber);
    }

    public double getX(){
        return stick.getX();
    }

    public double getY(){
        return stick.getY();
    }
}
