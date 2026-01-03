package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants.CameraConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;
import java.util.HashMap;

public class VisionSubsystem extends SubsystemBase {

    // Declare the position of the camera on the robot
    // Needed for apriltag localization
    Limelight3A limelight;
    Telemetry telemetry;
    int currentPipeline;

    public VisionSubsystem(Limelight3A limelight, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = limelight;
    }

    public double getApriltagTy(int id) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();
            for (LLResultTypes.FiducialResult detection : detections) {
                telemetry.addData("Id", detection.getFiducialId());
                if (detection.getFiducialId() == id) {
                    return detection.getTargetYDegrees();
                }
            }
        }
        return -1000;
    }

    public double getApriltagTx(int id) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();
            for (LLResultTypes.FiducialResult detection : detections) {
                telemetry.addData("Id", detection.getFiducialId());
                if (detection.getFiducialId() == id) {
                    return detection.getTargetXDegrees();
                }
            }
        }
        return -1000;
    }
    public void setApriltagPipeline() {
        currentPipeline = 1;
        limelight.pipelineSwitch(1);
    }
    public void setVisionPipeline() {
        currentPipeline = 0;
        limelight.pipelineSwitch(0);
    }
}
