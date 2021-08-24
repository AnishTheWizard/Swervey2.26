// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libs.Wrappers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. 
 * @author Anish Chandra
*/
public class Controller {
    private Joystick joy;

    public Controller(int port) {
        joy = new Joystick(port);
    }

    public double getAxis(int axis) {
        return joy.getRawAxis(axis);
    }

    public JoystickButton getButton(int btnNum) {
        return new JoystickButton(joy, btnNum);
    }

    public double getLeftJoyX() {
        return joy.getX();
    }

    public double getLeftJoyY() {
        return joy.getY();
    }

    public double getRightJoyX() {
        return joy.getRawAxis(4);
    }

    public double getRightJoyY() {
        return joy.getRawAxis(5);
    }

    public double getLeftTrigger() {
        return joy.getRawAxis(2);
    }

    public double getRightTrigger() {
        return joy.getRawAxis(3);
    }

    public JoystickButton getAButton() {
        return new JoystickButton(joy, 1);
    }

    public JoystickButton getBButton() {
        return new JoystickButton(joy, 2);
    }

    public JoystickButton getXButton() {
        return new JoystickButton(joy, 3);
    }

    public JoystickButton getYButton() {
        return new JoystickButton(joy, 4);
    }

    public JoystickButton getLBButton() {
        return new JoystickButton(joy, 5);
    }
    
    public JoystickButton getRBButton() {
        return new JoystickButton(joy, 6);
    }

    public JoystickButton getSTARTButton() {
        return new JoystickButton(joy, 7);
    }

    public JoystickButton getMENUButton() {
        return new JoystickButton(joy, 8);
    }

    public JoystickButton getLeftStickPress() {
        return new JoystickButton(joy, 9);
    }

    public JoystickButton getRightStickPress() {
        return new JoystickButton(joy, 10);
    }
}
