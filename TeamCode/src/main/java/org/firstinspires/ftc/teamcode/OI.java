package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class OI{
    GamepadEx driveGamepad, opGamepad;
    public OI(GamepadEx driveGamepad, GamepadEx opGamepad){
        this.driveGamepad = driveGamepad;
        this.opGamepad = opGamepad;
    }
    public Button dpadUp(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
    }
    public Button dpadDown(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);
    }
    public Button dpadLeft(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
    }
    public Button dpadRight(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT);
    }
    public Button a(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.A);
    }
    public Button b(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.B);
    }
    public Button x(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.X);
    }
    public Button y(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.Y);
    }
    public Trigger leftTrigger(GamepadEx gamepad) {
        return new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
    }
    public Trigger rightTrigger(GamepadEx gamepad) {
        return new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);
    }
    public Button leftBumper(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.LEFT_BUMPER);
    }
    public Button rightBumper(GamepadEx gamepad) {
        return new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
    }
}
