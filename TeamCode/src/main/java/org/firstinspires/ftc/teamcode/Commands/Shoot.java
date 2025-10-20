package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class Shoot extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    public Shoot(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;

        addRequirements(shooter);
        addRequirements(vision);
    }

    @Override
    public void execute() {
        if (vision.inShootingPosition()) {
            shooter.shoot();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
    }
}
