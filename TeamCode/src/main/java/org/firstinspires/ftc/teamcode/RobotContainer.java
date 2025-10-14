package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.*;

public class RobotContainer extends CommandOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    GamepadEx driveGamepad;
    GamepadEx opGamepad;
    Limelight3A ll3;
    public ChassisSubsystem chassis;

    @Override
    public void initialize() {
        // Put all of the things you want to initialize at the start of a match here
        // Like all hardware on a bot - motors, sensors, etc.
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackMotor");

        ll3 = hardwareMap.get(Limelight3A.class, "limelight");
        ChassisSubsystem chassis = new ChassisSubsystem(leftFront, rightFront, leftBack, rightBack);

        this.driveGamepad = new GamepadEx(gamepad1);
        this.opGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        // This is where we bind buttons to commands
        // This way of coding is better because it is more flexible

        chassis.setDefaultCommand(new RunCommand(() -> chassis.drive(
                driveGamepad.getLeftY(),
                driveGamepad.getLeftX(),
                driveGamepad.getRightX()
        ), chassis));
    }
}
