package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.*;

public class RobotContainer extends CommandOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    GamepadEx driveGamepad;
    GamepadEx opGamepad;
    public ChassisSubsystem chassis;

    @Override
    public void initialize() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackMotor");

        ChassisSubsystem chassis = new ChassisSubsystem(leftFront, rightFront, leftBack, rightBack);

        this.driveGamepad = new GamepadEx(gamepad1);
        this.opGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        chassis.setDefaultCommand(new RunCommand(() -> chassis.drive(
                driveGamepad.getLeftY(),
                driveGamepad.getLeftX(),
                driveGamepad.getRightX()
        ), chassis));
    }
}
