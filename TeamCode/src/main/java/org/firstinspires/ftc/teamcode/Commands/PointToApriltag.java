package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

public class PointToApriltag extends CommandBase {
    DriveSubsystem chassis;
    VisionSubsystem vision;
    int apriltag;
    DoubleSupplier forward;
    DoubleSupplier strafe;

    public PointToApriltag(DriveSubsystem chassis, VisionSubsystem limelight, int apriltag,
                           DoubleSupplier forward, DoubleSupplier strafe) {
        this.vision = limelight;
        this.chassis = chassis;
        this.apriltag = apriltag;
        this.forward = forward;
        this.strafe = strafe;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        vision.setApriltagPipeline();
    }

    @Override
    public void execute() {
        double rotateSpeed = 0;
        if (vision.getApriltagTy(apriltag) != -1000) {
            rotateSpeed = -vision.getApriltagTy(apriltag) * Constants.CameraConstants.P;
        }
        double forwardSpeed = forward.getAsDouble();
        double strafeSpeed = strafe.getAsDouble();

        chassis.drive(forwardSpeed, rotateSpeed, strafeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(0, 0, 0);
    }
}
