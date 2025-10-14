package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class Shoot extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final ChassisSubsystem chassis;

    public Shoot(ShooterSubsystem shooter, VisionSubsystem vision, ChassisSubsystem chassis) {
        this.shooter = shooter;
        this.vision = vision;
        this.chassis = chassis;

        addRequirements(shooter);
        addRequirements(vision);
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        if (vision.inShootingPosition()) {
            shooter.shoot();
        } else {
            // move to shooting position
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
    }
}
