package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp
public class ActualCode extends LinearOpMode {
    Initializer init = new Initializer();
    Limelight3A limelight;


    public void runOpMode() throws InterruptedException {
        DcMotor shooter;
        Servo gatekeeper;
        ShooterSubsystem shooterSubsystem;

        shooter = hardwareMap.get(DcMotor.class, Constants.Shooter);
        gatekeeper = hardwareMap.get(Servo.class, Constants.Indexer);

        shooterSubsystem = new ShooterSubsystem(shooter, gatekeeper);

        DcMotor intake;
        IntakeSubsystem intakeSubsystem;

        intake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.Intake);

        intakeSubsystem = new IntakeSubsystem(intake);

        DcMotor leftMotor, rightMotor;
        DcMotor leftFront, rightFront, leftBack, rightBack;
        DriveSubsystem chassis;

        IMU imu;
        imu = hardwareMap.get(IMU.class, Constants.IMU);

        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(hubOrientation));

        if (!Constants.mecanum) {
            leftMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.TankLeft);
            rightMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.TankRight);

            leftMotor.setDirection(DcMotor.Direction.REVERSE);

            chassis = new DriveSubsystem(leftMotor, rightMotor);
        }
        else {
            leftFront = hardwareMap.get(DcMotor.class, Constants.DriveConstants.LeftFront);
            rightFront = hardwareMap.get(DcMotor.class, Constants.DriveConstants.RightFront);
            leftBack = hardwareMap.get(DcMotor.class, Constants.DriveConstants.LeftBack);
            rightBack = hardwareMap.get(DcMotor.class, Constants.DriveConstants.RightBack);

            chassis = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);
            chassis.setUpIMU(imu);
        }

        limelight = hardwareMap.get(Limelight3A.class, Constants.Limelight);

        String color = "Red";

        waitForStart();
        limelight.pipelineSwitch(1);
        limelight.start();

        while (opModeIsActive()) {
            // Operator controls
            if (gamepad2.aWasPressed()) {
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
            if (gamepad2.left_trigger >= 0.5) {
                intakeSubsystem.startIntake();
            } else if (gamepad2.right_trigger >= 0.5) {
                intakeSubsystem.reverseIntake();
            } else {
                intakeSubsystem.stop();
            }

            // Studica servos go in the opposite direction as Gobilda ones... Why???
            if (gamepad2.b) {
                shooterSubsystem.openGate();
            } else {
                shooterSubsystem.closeGate();
            }

            // Driver controls
            if (gamepad1.aWasPressed()) {
                chassis.toggleSlowMode();
            }
            LLResult result = limelight.getLatestResult();
            Pose3D botPose;
            double forwardSpeed = -gamepad1.left_stick_y;
            double rotateSpeed = gamepad1.right_stick_x;
            double strafeSpeed = gamepad1.left_stick_x;

            if (result != null && result.isValid() && gamepad1.left_trigger > 0.5) {
                telemetry.addData("ty", result.getTy());
                botPose = result.getBotpose();
                telemetry.addData("Bot Position", botPose.getPosition());
                telemetry.addData("Bot Orientation", botPose.getOrientation());
                rotateSpeed = -result.getTy()*CameraConstants.P;
            }
            chassis.drive(forwardSpeed, rotateSpeed, strafeSpeed);


            telemetry.addData("Left Front", chassis.leftFrontMotor.getCurrentPosition());
            telemetry.addData("Left Back", chassis.leftBackMotor.getCurrentPosition());
            telemetry.addData("Right Front", chassis.rightFrontMotor.getCurrentPosition());
            telemetry.addData("Right Back", chassis.rightBackMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
