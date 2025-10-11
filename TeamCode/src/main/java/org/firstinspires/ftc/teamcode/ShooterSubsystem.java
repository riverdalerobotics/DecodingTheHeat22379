package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterSubsystem {
    public DcMotor flyWheel;
    public ShooterSubsystem (DcMotor flyWheel) {
        this.flyWheel = flyWheel;
    }
}
