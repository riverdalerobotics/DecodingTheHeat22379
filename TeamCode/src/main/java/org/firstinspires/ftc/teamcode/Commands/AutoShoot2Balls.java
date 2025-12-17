package org.firstinspires.ftc.teamcode.Commands;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class AutoShoot2Balls {

    private final ShooterSubsystem shooter;
    private final DriveSubsystem tank;

    public AutoShoot2Balls(ShooterSubsystem shooter, DriveSubsystem drive) {
        this.shooter = shooter;
        this.tank = drive;
    }

    public void runOpMode() throws InterruptedException {
        shooter.power = 0.9;
        shooter.closeGate();
        tank.drive(0.5, 0, 0);
        shooter.shoot();
        sleep(1700);
        tank.drive(0, 0, 0);
        sleep(1300);
        shooter.openGate();
        sleep(1500);
        shooter.stopShoot();
    }
}
