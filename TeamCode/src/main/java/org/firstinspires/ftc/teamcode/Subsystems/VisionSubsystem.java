package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    Limelight3A ll3;
    public VisionSubsystem(Limelight3A ll3) {
        this.ll3 = ll3;
    }

    public void start() {
        // Should run when the limelight is starting
        ll3.start();
        ll3.pipelineSwitch(0);
    }

    public void runAprilTag(String color) {

    }

    public void runArtifactDetection(String[] motif) {

    }

    public boolean inShootingPosition() {
        return false;
    }
}
