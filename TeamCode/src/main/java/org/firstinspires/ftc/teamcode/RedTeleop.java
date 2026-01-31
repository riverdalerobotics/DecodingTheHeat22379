package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;

//d = (h2-h1) / tan(a1+a2)
// a1 is 16degrees
// h1 is 17.4 cm
// h2 is ~30*2.54
// a2 is just tx
@TeleOp
public class RedTeleop extends CommandOpMode {
    DriveSubsystem chassis;
    ShooterSubsystem shooter;
    IntakeSubsystem intakeSubsystem;
    VisionSubsystem vision;
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor flyWheel;
    Servo gatekeeper;
    DcMotor intake;
    Limelight3A limelight;
    GamepadEx driveGamepad, opGamepad;
    VoltageSensor battery;
    IMU imu;
    OI oi;
    AdjustPower adjustPower;
    TurnToAngle pointForward, pointRight, pointLeft, pointBackward;
    ToggleSlowMode toggleSlowMode;
    PointToApriltag lookToTag;
    public void initializeSubsystems() {
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

        flyWheel = hardwareMap.get(DcMotor.class, ShooterConstants.Shooter);
        gatekeeper = hardwareMap.get(Servo.class, ShooterConstants.Indexer);
        shooter = new ShooterSubsystem(flyWheel, gatekeeper);

        intake = hardwareMap.get(DcMotor.class, IntakeConstants.Intake);
        intakeSubsystem = new IntakeSubsystem(intake);

        imu = hardwareMap.get(IMU.class, Constants.IMU);

        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(hubOrientation));
        chassis.setUpIMU(imu);

        driveGamepad = new GamepadEx(gamepad1);
        opGamepad = new GamepadEx(gamepad2);

        limelight = hardwareMap.get(Limelight3A.class, Constants.Limelight);
        vision = new VisionSubsystem(limelight, telemetry);
        battery = hardwareMap.voltageSensor.iterator().next();
        oi = new OI(driveGamepad, opGamepad);
    }

    @Override
    public void initialize() {
        initializeSubsystems();
        adjustPower = new AdjustPower(shooter, vision, 24, battery);
        toggleSlowMode = new ToggleSlowMode(chassis);
        lookToTag = new PointToApriltag(
                chassis,
                vision,
                24,
                driveGamepad::getLeftY,
                driveGamepad::getLeftX
        );
        pointForward = new TurnToAngle(chassis, 0);
        pointRight = new TurnToAngle(chassis, -90);
        pointLeft = new TurnToAngle(chassis, 90);
        pointBackward = new TurnToAngle(chassis, 180);

        register(chassis, shooter, intakeSubsystem, vision);
        limelight.start();

        oi.a(opGamepad).whenPressed(new InstantCommand(() -> shooter.toggle(), shooter));

        intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.stop(), intakeSubsystem));

        oi.x(driveGamepad).whenReleased(toggleSlowMode);
        oi.b(opGamepad).whenPressed(new RunCommand(() -> shooter.openGate(), shooter))
                .whenReleased(new RunCommand(() -> shooter.closeGate(), shooter));

        oi.dpadUp(driveGamepad).whenPressed(pointForward);
        oi.dpadDown(driveGamepad).whenPressed(pointBackward);
        oi.dpadLeft(driveGamepad).whenPressed(pointLeft);
        oi.dpadRight(driveGamepad).whenPressed(pointRight);
        oi.dpadUp(opGamepad).whenPressed(new InstantCommand(() -> shooter.faster(), shooter));
        oi.dpadDown(opGamepad).whenPressed(new InstantCommand(() -> shooter.slower(), shooter));

        oi.leftTrigger(opGamepad).whenActive(new RunCommand(() -> intakeSubsystem.startIntake(), intakeSubsystem))
                .whenInactive(new InstantCommand(() -> intakeSubsystem.stop(), intakeSubsystem));
        oi.rightTrigger(opGamepad).whileActiveContinuous(new InstantCommand(() -> intakeSubsystem.reverseIntake(), intakeSubsystem))
                .whenInactive(new InstantCommand(() -> intakeSubsystem.stop(), intakeSubsystem));

        oi.rightBumper(driveGamepad).whenReleased(new InstantCommand(() -> chassis.resetYaw()));

        chassis.setDefaultCommand(new RunCommand(() -> chassis.drive(
                driveGamepad.getLeftY(),
                driveGamepad.getRightX(),
                driveGamepad.getLeftX()
        ), chassis));

        oi.leftTrigger(driveGamepad).whileActiveOnce(lookToTag);
        oi.leftTrigger(driveGamepad).whileActiveOnce(adjustPower);
    }

    @Override
    public void run() {
        super.run();
        // Do NOT put anything in here unless you know what you are doing
        // Except telemetry

        telemetry.addData("Power", shooter.power);
        telemetry.update();
        if (shooter.isShooting) {
            shooter.shoot();
        }
    }
}