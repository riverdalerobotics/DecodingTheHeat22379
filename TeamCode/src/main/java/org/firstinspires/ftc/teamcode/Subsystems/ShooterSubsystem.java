package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterSubsystem extends SubsystemBase {
    public DcMotor flyWheel;
    public ShooterSubsystem (DcMotor flyWheel) {
        this.flyWheel = flyWheel;
        this.flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shoot () {
        flyWheel.setPower(1);
    }

    public void stopShoot () {
        flyWheel.setPower(0);
    }
}
