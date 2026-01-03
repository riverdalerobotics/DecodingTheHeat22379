package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public DcMotor flyWheel;
    public Servo gatekeeper;
    public boolean isShooting = false;
    public double power = 1;
    public ShooterSubsystem (DcMotor flyWheel, Servo gatekeeper) {
        this.gatekeeper = gatekeeper;
        this.flyWheel = flyWheel;
    }

    public void shoot () {
        flyWheel.setPower(power);
        isShooting = true;
    }

    public void stopShoot () {
        flyWheel.setPower(0);
        isShooting = false;
    }

    public void openGate() {
        gatekeeper.setPosition(Constants.ShooterConstants.servoClosedPosition);
    }
    public void closeGate() {
        gatekeeper.setPosition(Constants.ShooterConstants.servoOpenPosition);
    }

    public void faster() { power = Math.min(power + 0.05, 1); }
    public void slower() { power = Math.max(power - 0.05, 0.5); }
    public void setPower(double power) { this.power = Math.max(-1, Math.min(power, 1));}

    public void toggle() {
        if (isShooting) {
            stopShoot();
        } else {
            shoot();
        }
    }
}
