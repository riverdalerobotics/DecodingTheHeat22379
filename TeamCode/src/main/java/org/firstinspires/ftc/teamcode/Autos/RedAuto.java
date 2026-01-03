package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.AutoShoot2Balls;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@Autonomous
public class RedAuto extends LinearOpMode {

    DcMotor shooter;
    DcMotor intake;
    DcMotor leftMotor, rightMotor;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DriveSubsystem chassis;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    Servo gatekeeper;
    IMU imu;

    public void runOpMode() throws InterruptedException {
        // Initialization code: put at the start of every runOpMode
        gatekeeper = hardwareMap.get(Servo.class, ShooterConstants.Indexer);

        shooter = hardwareMap.get(DcMotor.class, ShooterConstants.Shooter);
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
        // End of initialization code

        AutoShoot2Balls autoShoot2Balls = new AutoShoot2Balls(shooterSubsystem, chassis, intakeSubsystem);

        // Reminds me of FLL
        waitForStart();
        autoShoot2Balls.runOpMode();
        chassis.forwardByTicks(400);
        chassis.rotateToAngle(-36);
        chassis.strafeByTicks(-1500);
        intakeSubsystem.startIntake();
        chassis.forwardByTicks(1000);
        intakeSubsystem.stop();
        chassis.drive(0, 0, 0);
    }
}
