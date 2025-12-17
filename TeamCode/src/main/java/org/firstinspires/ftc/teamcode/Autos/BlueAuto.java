package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.AutoShoot2Balls;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@Autonomous
public class BlueAuto extends LinearOpMode {

    DcMotor shooter;
    DcMotor leftMotor, rightMotor;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DriveSubsystem chassis;
    ShooterSubsystem shooterSubsystem;
    Servo gatekeeper;

    public void runOpMode() throws InterruptedException {
        gatekeeper = hardwareMap.get(Servo.class, Constants.Indexer);

        shooter = hardwareMap.get(DcMotor.class, Constants.Shooter);
        shooterSubsystem = new ShooterSubsystem(shooter, gatekeeper);

        if (!Constants.mecanum) {
            leftMotor = hardwareMap.get(DcMotor.class, Constants.TankLeft);
            rightMotor = hardwareMap.get(DcMotor.class, Constants.TankRight);

            leftMotor.setDirection(DcMotor.Direction.REVERSE);

            chassis = new DriveSubsystem(leftMotor, rightMotor);
        } else {
            leftFront = hardwareMap.get(DcMotor.class, Constants.LeftFront);
            rightFront = hardwareMap.get(DcMotor.class, Constants.RightFront);
            leftBack = hardwareMap.get(DcMotor.class, Constants.LeftBack);
            rightBack = hardwareMap.get(DcMotor.class, Constants.RightBack);

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);

            chassis = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);
        }

        AutoShoot2Balls autoShoot2Balls = new AutoShoot2Balls(shooterSubsystem, chassis);

        // Reminds me of FLL
        waitForStart();
        autoShoot2Balls.runOpMode();
        chassis.drive(-0.5, 0, 0);
        sleep(500);
        chassis.drive(0, -0.5, 0);
        sleep(400);
        chassis.drive(-0.5, 0, 0);
        sleep(1000);
        chassis.drive(0, 0, 0);
    }
}
