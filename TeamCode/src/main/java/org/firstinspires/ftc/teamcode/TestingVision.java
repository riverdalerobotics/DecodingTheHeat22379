package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

@TeleOp
public class TestingVision extends LinearOpMode {

    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        VisionSubsystem vision = new VisionSubsystem(limelight, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Detections", vision);
            telemetry.update();
        }
    }

}
