package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@TeleOp
public class ActualCode extends LinearOpMode {
    DcMotor shooter;
    DcMotor leftMotor, rightMotor;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor intake;
    DriveSubsystem chassis;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    Servo gatekeeper;

    public void runOpMode() throws InterruptedException {
        gatekeeper = hardwareMap.get(Servo.class, Constants.Indexer);
        
        shooter = hardwareMap.get(DcMotor.class, Constants.Shooter);
        shooterSubsystem = new ShooterSubsystem(shooter, gatekeeper);

        intake = hardwareMap.get(DcMotor.class, Constants.Intake);

        intakeSubsystem = new IntakeSubsystem(intake);

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
        String color = "Red";

        while (!opModeIsActive()) {
            if (gamepad1.xWasPressed() && !isStopRequested()) {
                color = (color.equals("Red")) ? "Blue" : "Red";
            }
            telemetry.addData("Alliance", color);
        }

        while (opModeIsActive()) {
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
                gatekeeper.setPosition(Constants.servoClosedPosition);
            } else {
                gatekeeper.setPosition(Constants.servoOpenPosition);
            }
            telemetry.addData("Gatekeeper Position", gatekeeper.getPosition());
            telemetry.addData("Motor Speed", shooterSubsystem.power);
            telemetry.update();

            chassis.drive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        }
    }
}
