package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

import java.util.List;

@TeleOp
@Configurable
public class ActualCode extends LinearOpMode {
    Limelight3A limelight;
    VisionSubsystem vision;
    VoltageSensor battery;
    DcMotor shooter;
    Servo gatekeeper;
    ShooterSubsystem shooterSubsystem;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DriveSubsystem chassis;
    IMU imu;
    String color = "Blue";

    public void runOpMode() {
        shooter = hardwareMap.get(DcMotor.class, ShooterConstants.Shooter);
        gatekeeper = hardwareMap.get(Servo.class, ShooterConstants.Indexer);

        shooterSubsystem = new ShooterSubsystem(shooter, gatekeeper);

        DcMotor intake;
        IntakeSubsystem intakeSubsystem;

        intake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.Intake);

        intakeSubsystem = new IntakeSubsystem(intake);

        imu = hardwareMap.get(IMU.class, Constants.IMU);

        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(hubOrientation));

        leftFront = hardwareMap.get(DcMotor.class, Constants.DriveConstants.LeftFront);
        rightFront = hardwareMap.get(DcMotor.class, Constants.DriveConstants.RightFront);
        leftBack = hardwareMap.get(DcMotor.class, Constants.DriveConstants.LeftBack);
        rightBack = hardwareMap.get(DcMotor.class, Constants.DriveConstants.RightBack);

        chassis = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);
        chassis.setUpIMU(imu);


        limelight = hardwareMap.get(Limelight3A.class, Constants.Limelight);
        battery = hardwareMap.voltageSensor.iterator().next();
        vision = new VisionSubsystem(limelight, telemetry);

        waitForStart();
        imu.resetYaw();
        limelight.pipelineSwitch(1);
        limelight.start();

        while (opModeIsActive()) {
            // Operator controls
            if (gamepad2.aWasPressed() || gamepad1.aWasPressed()) {
                shooterSubsystem.toggle();
            }
            if (gamepad2.dpadUpWasPressed()) {
                shooterSubsystem.faster();
                if (shooterSubsystem.isShooting) {
                    shooterSubsystem.shoot();
                }
            } if (gamepad2.dpadDownWasPressed()) {
                shooterSubsystem.slower();
                if (shooterSubsystem.isShooting) {
                    shooterSubsystem.shoot();
                }
            }
            if (gamepad2.left_trigger >= 0.5 || gamepad1.left_bumper) {
                intakeSubsystem.startIntake();
            } else if (gamepad2.right_trigger >= 0.5 || gamepad1.right_bumper) {
                intakeSubsystem.reverseIntake();
            } else {
                intakeSubsystem.stop();
            }

            // Studica servos go in the opposite direction as Gobilda ones... Why???
            if (gamepad2.b || gamepad1.b) {
                shooterSubsystem.openGate();
            } else {
                shooterSubsystem.closeGate();
            }

            // Driver controls
            if (gamepad1.xWasPressed()) {
                chassis.toggleSlowMode();
            }

            if (gamepad1.left_trigger > 0.1) {
                limelight.pipelineSwitch(1);
            } else if (gamepad1.right_trigger > 0.1) {
                limelight.pipelineSwitch(0);
            }
            if (gamepad1.dpadUpWasPressed()) {
                chassis.rotateToAngle(0);
            } else if (gamepad1.dpadDownWasPressed()) {
                chassis.rotateToAngle(180);
            } else if (gamepad1.dpadLeftWasPressed()) {
                chassis.rotateToAngle(90);
            } else if (gamepad1.dpadRightWasPressed()) {
                chassis.rotateToAngle(-90);
            }

            LLResult result = limelight.getLatestResult();

            double forwardSpeed = -gamepad1.left_stick_y;
            double rotateSpeed = gamepad1.right_stick_x;
            double strafeSpeed = gamepad1.left_stick_x;

            if (result != null && result.isValid() && gamepad1.left_trigger > 0.5) {
                // Look at the apriltag code
                rotateSpeed = -result.getTy()*CameraConstants.P;
                if (Math.abs(rotateSpeed) < DriveConstants.rotationStallSpeed) {
                    rotateSpeed = Math.copySign(DriveConstants.rotationStallSpeed, rotateSpeed);
                }
                double d = getDist(result);
                double power = ShooterConstants.startingPower -
                        (battery.getVoltage()-12)*ShooterConstants.batteryModifier + d*ShooterConstants.distanceModifier;

                telemetry.addData("Distance", d);
                shooterSubsystem.setPower(power);
                if (Math.abs(result.getTy()) < 2 && power < 1.5) shooterSubsystem.shoot();
                else shooterSubsystem.stopShoot();
            } else if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger < 0.1 && result != null && result.isValid()) {
                List<LLResultTypes.DetectorResult> results = result.getDetectorResults();
                for (LLResultTypes.DetectorResult ball : results) {
                    telemetry.addData("Ball", ball.getTargetCorners());
                    List<List<Double>> corners = ball.getTargetCorners();

                }
            }

            chassis.drive(forwardSpeed, rotateSpeed, strafeSpeed);
            telemetry.addData("Power", shooterSubsystem.power);
            telemetry.addData("IsShooting", shooterSubsystem.isShooting);
            telemetry.update();
        }
    }

    public double getDist(LLResult result) {
        // Never forget how the Limelight is rotated 90 degrees.

        double angleFromBottom = 3.14159;

        if (!result.getDetectorResults().isEmpty()) {
            List<LLResultTypes.DetectorResult> results = result.getDetectorResults();
            angleFromBottom = 1000;
            for (LLResultTypes.DetectorResult ball : results) {
                List<List<Double>> corners = ball.getTargetCorners();
                double center = (corners.get(1).get(0) + corners.get(3).get(0))/2;
                double top = corners.get(1).get(0);
                angleFromBottom = Math.min(angleFromBottom, top / CameraConstants.horPixelPerDeg);
            }
        } else if (!result.getFiducialResults().isEmpty()) {
            List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();
            for (LLResultTypes.FiducialResult detection : detections) {
                if ((detection.getFiducialId() == 20 && color.equals("Blue")) ||
                        (detection.getFiducialId() == 24 && color.equals("Red"))) {
                    angleFromBottom = detection.getTargetXDegrees();
                }
            }
        } if (angleFromBottom == 3.14159) {
            return -1000;
        }
        double d = (CameraConstants.apriltagHeight-CameraConstants.height) /
                Math.tan(CameraConstants.pitch + Math.toRadians(angleFromBottom));
        return d;
    }

}
