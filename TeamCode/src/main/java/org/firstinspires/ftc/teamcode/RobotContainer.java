package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp
public class RobotContainer extends CommandOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor flyWheel;
    Servo gatekeeper;
    DcMotor intake1;
    DcMotor intake2;
    GamepadEx driveGamepad;
    GamepadEx opGamepad;
    public ChassisSubsystem chassis;
    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    @Override
    public void initialize() {
        // Put all of the things you want to initialize at the start of a match here
        // Like all hardware on a bot - motors, sensors, etc.
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackMotor");
        chassis = new ChassisSubsystem(leftFront, rightFront, leftBack, rightBack);

//        flyWheel = hardwareMap.get(DcMotor.class, "flywheel");
//        gatekeeper = hardwareMap.get(Servo.class, "gatekeeper");
//        shooter = new ShooterSubsystem(flyWheel, gatekeeper);
//
//        intake1 = hardwareMap.get(DcMotor.class, "intake1");
//        intake2 = hardwareMap.get(DcMotor.class, "intake2");
//        intake = new IntakeSubsystem(intake1, intake2);

        this.driveGamepad = new GamepadEx(gamepad1);
//        this.opGamepad = new GamepadEx(gamepad2);
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
                driveGamepad.getLeftX(),
                driveGamepad.getRightX()
        ), chassis));

//        new GamepadButton(opGamepad, GamepadKeys.Button.A)
//                .whenPressed(new RunCommand(() -> shooter.shoot(), shooter))
//                .whenReleased(new RunCommand(() -> shooter.stopShoot(), shooter));
//
    }
}
