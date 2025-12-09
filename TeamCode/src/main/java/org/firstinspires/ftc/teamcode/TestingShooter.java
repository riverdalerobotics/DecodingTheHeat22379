package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TankSubsystem;

@TeleOp
public class TestingShooter extends LinearOpMode {
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
        shooterSubsystem = new ShooterSubsystem(shooter);

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
            if (gamepad1.aWasPressed()) {
                shooterSubsystem.toggle();
            } if (gamepad1.yWasPressed()) {
                shooterSubsystem.faster();
            } if (gamepad1.xWasPressed()) {
                shooterSubsystem.slower();
            } if (gamepad2.a) {

            }

            if (gamepad1.b) {
                gatekeeper.setPosition(0.13);
            } else {
                gatekeeper.setPosition(0.3);
            }
            telemetry.addData("Gatekeeper Position", gatekeeper.getPosition());
            telemetry.update();

            if (driving) {
                chassis.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
        }
    }
}
