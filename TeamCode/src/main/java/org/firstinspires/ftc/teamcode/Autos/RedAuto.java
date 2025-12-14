package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TankSubsystem;

@Autonomous
public class RedAuto extends LinearOpMode {

    DcMotor shooter;
    DcMotor leftMotor;
    DcMotor rightMotor;
    TankSubsystem chassis;
    ShooterSubsystem shooterSubsystem;
    Servo gatekeeper;
    boolean driving = true;

    public void runOpMode() throws InterruptedException {
        gatekeeper = hardwareMap.get(Servo.class, "gatekeeper");

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooterSubsystem = new ShooterSubsystem(shooter, gatekeeper);

        if (driving) {
            leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
            rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

            leftMotor.setDirection(DcMotor.Direction.REVERSE);

            chassis = new TankSubsystem(leftMotor, rightMotor);
        }

        // Reminds me of FLL
        waitForStart();
        ShooterSubsystem.power = 0.85;
        shooterSubsystem.closeGate();
        chassis.drive(0.5, 0);
        shooterSubsystem.shoot();
        sleep(1500);
        chassis.drive(0, 0);
        sleep(1300);
        shooterSubsystem.openGate();
        sleep(1500);
        shooterSubsystem.stopShoot();
        chassis.drive(-0.5, 0);
        sleep(500);
        chassis.drive(0, 0.5);
        sleep(400);
        chassis.drive(-0.5, 0);
        sleep(1000);
        chassis.drive(0, 0);
    }
}