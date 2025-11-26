package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp
public class TestingShooter extends LinearOpMode {
    DcMotor shooter;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    ChassisSubsystem chassis;
    ShooterSubsystem shooterSubsystem;
    boolean driving = false;

    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooterSubsystem = new ShooterSubsystem(shooter);

        if (driving) {
            frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFrontMotor");
            frontRightDrive = hardwareMap.get(DcMotor.class, "leftBackMotor");
            backLeftDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");
            backRightDrive = hardwareMap.get(DcMotor.class, "rightBackMotor");

            // We set the left motors in reverse which is needed for drive trains where the left
            // motors are opposite to the right ones.
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

            // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
            // wires, you should remove these
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            chassis = new ChassisSubsystem(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        }
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                shooterSubsystem.toggle();
            }

            if (driving) {
                chassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }
    }
}
