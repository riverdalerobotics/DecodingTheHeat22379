package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.*;

public class ApriltagDriveToPosition extends CommandBase {
    MecanumSubsystem chassis;
    VisionSubsystem vision;

    public ApriltagDriveToPosition (MecanumSubsystem chassis, VisionSubsystem vision) {
        this.vision = vision;
        this.chassis = chassis;
    }

    @Override
    public void execute() {

    }
}
