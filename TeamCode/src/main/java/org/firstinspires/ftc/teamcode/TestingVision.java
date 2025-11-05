package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

@TeleOp
public class TestingVision extends LinearOpMode {

    String cameraName = "camera";
    VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName camera = hardwareMap.get(WebcamName.class, "camera");
        vision = new VisionSubsystem(camera, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Detections", vision.runArtifactDetection());
            telemetry.update();
        }
    }

}
