package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class ToggleSlowMode extends CommandBase {
    DriveSubsystem chassis;
    public ToggleSlowMode(DriveSubsystem chassis) {
        this.chassis = chassis;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.toggleSlowMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
