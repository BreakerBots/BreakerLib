// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Wrapper for Xinput controllers. */
public class BreakerXboxController {

    // Xbox digital button ports
    public static final int A_PORT = 1;
    public static final int B_PORT = 2;
    public static final int X_PORT = 3;
    public static final int Y_PORT = 4;
    public static final int L_BUMP_PORT = 5;
    public static final int R_BUMP_PORT = 6;
    public static final int BACK_PORT = 7;
    public static final int START_PORT = 8;
    public static final int L_STICK_PRESS_PORT = 9;
    public static final int R_STICK_PRESS_PORT = 10;
    // Xbox axis ports
    public static final int LEFT_X_PORT = 0;
    public static final int LEFT_Y_PORT = 1;
    public static final int L_TRIGGER_PORT = 2;
    public static final int R_TRIGGER_PORT = 3;
    public static final int RIGHT_X_PORT = 4;
    public static final int RIGHT_Y_PORT = 5;
    // Xbox D-Pad angle constants
    public static final int DPAD_UP_ANG = 0;
    public static final int DPAD_RIGHT_ANG = 90;
    public static final int DPAD_DOWN_ANG = 180;
    public static final int DPAD_LEFT_ANG = 270;

    private XboxController controller;

    private JoystickButton buttonA;
    private JoystickButton buttonB;
    private JoystickButton buttonX;
    private JoystickButton buttonY;
    private JoystickButton leftBumper;
    private JoystickButton rightBumper;
    private JoystickButton leftStickButton;
    private JoystickButton rightStickButton;
    private JoystickButton startButton;
    private JoystickButton backButton;

    private POVButton dPadUp;
    private POVButton dPadDown;
    private POVButton dPadLeft;
    private POVButton dPadRight;

    public BreakerXboxController(int xboxPortNum) {
        controller = new XboxController(xboxPortNum);
        buttonA = new JoystickButton(controller, A_PORT);
        buttonB = new JoystickButton(controller, B_PORT);
        buttonX = new JoystickButton(controller, X_PORT);
        buttonY = new JoystickButton(controller, Y_PORT);
        leftBumper = new JoystickButton(controller, L_BUMP_PORT);
        rightBumper = new JoystickButton(controller, R_BUMP_PORT);
        leftStickButton = new JoystickButton(controller, L_STICK_PRESS_PORT);
        rightStickButton = new JoystickButton(controller, R_STICK_PRESS_PORT);
        startButton = new JoystickButton(controller, START_PORT);
        backButton = new JoystickButton(controller, BACK_PORT);

        dPadUp = new POVButton(controller, DPAD_UP_ANG);
        dPadDown = new POVButton(controller, DPAD_DOWN_ANG);
        dPadLeft = new POVButton(controller, DPAD_LEFT_ANG);
        dPadRight = new POVButton(controller, DPAD_RIGHT_ANG);
    }

    public JoystickButton getBackButton() {
        return backButton;
    }

    public JoystickButton getButtonA() {
        return buttonA;
    }

    public JoystickButton getButtonB() {
        return buttonB;
    }

    public JoystickButton getButtonX() {
        return buttonX;
    }

    public JoystickButton getButtonY() {
        return buttonY;
    }

    public XboxController getBaseController() {
        return controller;
    }

    public JoystickButton getLeftBumper() {
        return leftBumper;
    }

    public JoystickButton getLeftStickButton() {
        return leftStickButton;
    }

    public JoystickButton getRightBumper() {
        return rightBumper;
    }

    public JoystickButton getRightStickButton() {
        return rightStickButton;
    }

    public JoystickButton getStartButton() {
        return startButton;
    }

    public POVButton getdPadDown() {
        return dPadDown;
    }

    public POVButton getdPadLeft() {
        return dPadLeft;
    }

    public POVButton getdPadRight() {
        return dPadRight;
    }

    public POVButton getdPadUP() {
        return dPadUp;
    }
}
