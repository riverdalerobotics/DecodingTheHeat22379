package org.firstinspires.ftc.teamcode.Commands;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class AutoShoot3Balls {

    private final ShooterSubsystem shooter;
    private final DriveSubsystem tank;
    private final IntakeSubsystem intake;

    public AutoShoot3Balls(ShooterSubsystem shooter, DriveSubsystem drive, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.tank = drive;
        this.intake = intake;
    }

    public void runOpMode() throws InterruptedException {
        shooter.power = 0.62;
        tank.resetYaw();
        shooter.shoot();
        tank.forwardByTicks(-1100);
        sleep(1500);
        shooter.openGate();
        sleep(1500);
        intake.intake.setPower(-0.8);
        sleep(3000);
        intake.stop();
        shooter.stopShoot();
        shooter.closeGate();
    }
}
