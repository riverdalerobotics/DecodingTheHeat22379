package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class Initializer extends LinearOpMode {
    DcMotor leftMotor, rightMotor;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DriveSubsystem chassis;
    DcMotor shooter;
    Servo gatekeeper;
    ShooterSubsystem shooterSubsystem;
    DcMotor intake;
    IntakeSubsystem intakeSubsystem;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException{

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

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);

            chassis = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);
            chassis.setUpIMU(imu);
        }
        shooter = hardwareMap.get(DcMotor.class, Constants.Shooter);
        gatekeeper = hardwareMap.get(Servo.class, Constants.Indexer);

        shooterSubsystem = new ShooterSubsystem(shooter, gatekeeper);

        intake = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.Intake);

        intakeSubsystem = new IntakeSubsystem(intake);
    }

    public DriveSubsystem initDrive() {
        return chassis;
    }

    public ShooterSubsystem initShooter() {
        return shooterSubsystem;
    }

    public IntakeSubsystem initIntake() {

        return intakeSubsystem;
    }
}
