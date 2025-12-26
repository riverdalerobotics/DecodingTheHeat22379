package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Subsystems.*;

public class PointToApriltagTimed {
    MecanumSubsystem chassis;
    Limelight3A limelight;

    public PointToApriltagTimed(MecanumSubsystem chassis, Limelight3A limelight) {
        this.limelight = limelight;
        this.chassis = chassis;
    }

    public void runOpMode() {

    }
}
