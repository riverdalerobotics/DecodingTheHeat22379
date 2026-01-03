package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase {
    DriveSubsystem chassis;
    double targetAngle;
    int timeLimit = 3000;
    long endTime;
    public TurnToAngle(DriveSubsystem chassis, double angle) {
        this.chassis = chassis;
        this.targetAngle = angle;

        endTime = System.currentTimeMillis() + timeLimit;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        double currentAngle = chassis.getYaw();
        double error = targetAngle - currentAngle;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        // 4. CALCULATE POWER USING A P-CONTROLLER
        // Proportional control slows the robot down as it nears the target.
        double rotatePower = error * Constants.DriveConstants.autoTurnP;

        // 5. ADD STALL SPEED AND CLAMP THE POWER
        // Ensure we always have enough power to move, but never exceed 1.0.
        if (Math.abs(rotatePower) < Constants.DriveConstants.rotationStallSpeed) {
            rotatePower = Math.copySign(Constants.DriveConstants.rotationStallSpeed, rotatePower);
        }
        rotatePower = Math.max(-1, Math.min(1, rotatePower)); // Clamp between -1.0 and 1.0

        // 6. CALL THE DRIVE FUNCTION
        chassis.drive(0, -rotatePower, 0);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = chassis.getYaw();
        double error = targetAngle - currentAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        return Math.abs(error) < 1.0 || System.currentTimeMillis() > endTime;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(0, 0, 0);
    }
}
