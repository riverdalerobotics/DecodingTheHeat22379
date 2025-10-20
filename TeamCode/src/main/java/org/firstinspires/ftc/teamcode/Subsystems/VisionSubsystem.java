package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Commands.ApriltagDriveToPosition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.teamcode.Constants.*;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    VisionPortal portal;
    ColorBlobLocatorProcessor greenBalls;
    ColorBlobLocatorProcessor purpleBalls;
    AprilTagProcessor apriltag;
    Telemetry telemetry;
    Pose2D location;

    // Declare the position of the camera on the robot
    // Needed for apriltag localization
    private final Position cameraPosition = new Position(DistanceUnit.MM,
            CameraConstants.x, CameraConstants.y, CameraConstants.z, 0); // Important! Find values for this and put in constants
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            CameraConstants.yaw, CameraConstants.pitch, CameraConstants.roll, 0); // Important! Find these too
    public VisionSubsystem(WebcamName webcam, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create the artifact locators
        greenBalls = buildColor(ColorRange.ARTIFACT_GREEN);
        purpleBalls = buildColor(ColorRange.ARTIFACT_PURPLE);

        // Create the apriltag detector
        apriltag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // Declare the camera
        portal = new VisionPortal.Builder()
                .addProcessor(greenBalls)
                .addProcessor(purpleBalls)
                .addProcessor(apriltag)
                .setCameraResolution(new Size(640, 480))
                .setCamera(webcam)
                .build();
    }

    // Method used to build blob locators for various coloUrs
    public ColorBlobLocatorProcessor buildColor(ColorRange color) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(0, 255, 0))
                .setBlurSize(5)

                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        List<AprilTagDetection> detections = apriltag.getDetections();

        for (AprilTagDetection tag : detections) {
            if (tag.metadata != null) {
                telemetry.addData("Tag data", tag.metadata);
            }
        }
        return detections;
    }

    public void runArtifactDetection(String[] motif) {

    }

    public boolean inShootingPosition() {
        return false;
    }
}
