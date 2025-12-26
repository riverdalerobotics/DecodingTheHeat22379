package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Constants.*;


@TeleOp
public class RobotContainer extends CommandOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor flyWheel;
    Servo gatekeeper;
    DcMotor intake;
    GamepadEx driveGamepad;
    GamepadEx opGamepad;
    public DriveSubsystem chassis;
    public ShooterSubsystem shooter;
    public IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        // Put all of the things you want to initialize at the start of a match here
        // Like all hardware on a bot - motors, sensors, etc.
        leftFront = hardwareMap.get(DcMotor.class, DriveConstants.LeftFront);
        rightFront = hardwareMap.get(DcMotor.class, DriveConstants.RightFront);
        leftBack = hardwareMap.get(DcMotor.class, DriveConstants.LeftBack);
        rightBack = hardwareMap.get(DcMotor.class, DriveConstants.RightBack);
        chassis = new DriveSubsystem(leftFront, rightFront, leftBack, rightBack);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flyWheel = hardwareMap.get(DcMotor.class, Constants.Shooter);
        gatekeeper = hardwareMap.get(Servo.class, Constants.Indexer);
        shooter = new ShooterSubsystem(flyWheel, gatekeeper);

        intake = hardwareMap.get(DcMotor.class, IntakeConstants.Intake);
        intakeSubsystem = new IntakeSubsystem(intake);

        this.driveGamepad = new GamepadEx(gamepad1);
        this.opGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Hi", "Hello");
        // This is where we bind buttons to commands
        // This way of coding is better because it is more flexible
        // Each of these

        chassis.setDefaultCommand(new RunCommand(() -> chassis.drive(
                driveGamepad.getLeftY(),
                driveGamepad.getRightX(),
                driveGamepad.getLeftX()
        ), chassis));

        new GamepadButton(opGamepad, GamepadKeys.Button.A)
                .whenPressed(new RunCommand(() -> shooter.toggle(), shooter));

        new GamepadButton(opGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                .whenPressed(new RunCommand(() -> intakeSubsystem.startIntake(), intakeSubsystem))
                .whenReleased(new RunCommand(() -> intakeSubsystem.stop(), intakeSubsystem));

    }
}
