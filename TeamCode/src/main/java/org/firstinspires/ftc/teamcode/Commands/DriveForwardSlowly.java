package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry; // <-- Import Telemetry
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class DriveForwardSlowly extends CommandBase {

    private final DriveSubsystem chassis;
    private final double slowSpeed;
    private final Telemetry telemetry; // <-- Add Telemetry variable

    /**
     * Creates a new DriveForwardSlowly command.
     * @param chassis The DriveSubsystem to be used.
     * @param speed The desired forward speed (e.g., 0.2).
     * @param telemetry The Telemetry object for output.  // <-- Add Telemetry to Javadoc
     */
    public DriveForwardSlowly(DriveSubsystem chassis, double speed, Telemetry telemetry) { // <-- Add Telemetry to constructor
        this.chassis = chassis;
        this.slowSpeed = speed;
        this.telemetry = telemetry; // <-- Store the telemetry object
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        telemetry.addData("Command Status", "DriveForwardSlowly: INITIALIZED");
        telemetry.update();
    }

    @Override
    public void execute() {
        chassis.drive(slowSpeed, 0, 0);
        // Add telemetry that updates continuously while the command is running
        telemetry.addData("Command Status", "DriveForwardSlowly: EXECUTING");
        telemetry.addData("Driving Speed", slowSpeed);
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(0, 0, 0);
        telemetry.addData("Command Status", "DriveForwardSlowly: ENDED");
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
