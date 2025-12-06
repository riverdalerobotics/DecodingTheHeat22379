package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem extends SubsystemBase {
    public DcMotor flyWheel;
    public Servo gatekeeper;
    public boolean isShooting = false;
    public ShooterSubsystem (DcMotor flyWheel) {
        this.flyWheel = flyWheel;
    }

    public void shoot () {
        flyWheel.setPower(-1);
    }

    public void stopShoot () {
        flyWheel.setPower(0);
    }

    public void toggle() {
        if (isShooting) {
            stopShoot();
        } else {
            shoot();
        }
        isShooting = !isShooting;
    }
}
