package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.AdjustPower;
import org.firstinspires.ftc.teamcode.Commands.PointToApriltag;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.OI;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

@Autonomous
public class RedFarAuto extends LinearOpMode {
    DcMotor shooter;
    DcMotor intake;
    DcMotor leftMotor, rightMotor;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DriveSubsystem chassis;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    VisionSubsystem vision;
    Servo gatekeeper;
    IMU imu;
    Limelight3A ll;

    public void runOpMode() throws InterruptedException {
        // Initialization code: put at the start of every runOpMode
        gatekeeper = hardwareMap.get(Servo.class, Constants.ShooterConstants.Indexer);

        shooter = hardwareMap.get(DcMotor.class, Constants.ShooterConstants.Shooter);
        shooterSubsystem = new ShooterSubsystem(shooter, gatekeeper);

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

        intake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.Intake);
        intakeSubsystem = new IntakeSubsystem(intake);

        ll = hardwareMap.get(Limelight3A.class, Constants.Limelight);
        vision = new VisionSubsystem(ll, telemetry);

        // End of initialization code

        PointToApriltag point = new PointToApriltag(chassis, vision, 24, 2000);
        AdjustPower adjust = new AdjustPower(shooterSubsystem, vision, 24, hardwareMap.voltageSensor.iterator().next());
        shooterSubsystem.shoot();

        waitForStart();
        point.initialize();
        while (!point.isFinished()) {
            point.execute();
            adjust.execute();
        }
        sleep(1000);
        shooterSubsystem.openGate();
        sleep(1000);
        intakeSubsystem.intake.setPower(-0.8);
        sleep(4000);
        intakeSubsystem.stop();
        shooterSubsystem.closeGate();
        chassis.drive(-0.5, 0, 0);
        sleep(500);
        chassis.drive(0, 0, 0);
    }
}
