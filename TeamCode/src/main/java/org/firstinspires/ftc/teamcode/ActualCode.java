package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TankSubsystem;

@TeleOp
public class ActualCode extends LinearOpMode {
    DcMotor shooter;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor intake1;
    DcMotor intake2;
    TankSubsystem chassis;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    Servo gatekeeper;
    boolean driving = true;

    public void runOpMode() throws InterruptedException {
        gatekeeper = hardwareMap.get(Servo.class, "gatekeeper");
        
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooterSubsystem = new ShooterSubsystem(shooter, gatekeeper);

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        intakeSubsystem = new IntakeSubsystem(intake1, intake2);

        if (driving) {
            leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
            rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

            leftMotor.setDirection(DcMotor.Direction.REVERSE);

            chassis = new TankSubsystem(leftMotor, rightMotor);
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

            if (driving) {
                chassis.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
        }
    }
}
