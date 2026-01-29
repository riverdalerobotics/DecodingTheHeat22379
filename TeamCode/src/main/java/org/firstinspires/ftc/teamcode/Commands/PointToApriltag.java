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
    long timeLimit = 300000, endTime;
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

    public PointToApriltag(DriveSubsystem chassis, VisionSubsystem limelight,
                           int apriltag, long timeLimit) {
        this.vision = limelight;
        this.chassis = chassis;
        this.apriltag = apriltag;
        this.timeLimit = timeLimit;
        addRequirements(chassis);
    }


    @Override
    public void initialize() {
        vision.setApriltagPipeline();
        endTime = System.currentTimeMillis() + timeLimit;
    }

    @Override
    public void execute() {
        double rotateSpeed = 0;
        if (vision.getApriltagTy(apriltag) != -1000) {
            rotateSpeed = (-vision.getApriltagTy(apriltag)-1) * Constants.CameraConstants.P;
        }
        if (forward != null && strafe != null) {
            double forwardSpeed = forward.getAsDouble();
            double strafeSpeed = strafe.getAsDouble();

            chassis.drive(forwardSpeed, rotateSpeed, strafeSpeed);
        } else {
            chassis.drive(0, rotateSpeed, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > endTime;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(0, 0, 0);
    }
}
