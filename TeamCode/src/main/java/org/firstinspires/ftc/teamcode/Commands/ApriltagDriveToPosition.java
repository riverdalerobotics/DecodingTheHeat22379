package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.*;

public class ApriltagDriveToPosition extends CommandBase {
    ChassisSubsystem chassis;
    VisionSubsystem vision;

    public ApriltagDriveToPosition (ChassisSubsystem chassis, VisionSubsystem vision) {
        this.vision = vision;
        this.chassis = chassis;
    }

    @Override
    public void execute() {

    }
}
