package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem extends SubsystemBase {
    public DcMotor intake1;
    public DcMotor intake2;
    public IntakeSubsystem (DcMotor intake1, DcMotor intake2) {
        this.intake1 = intake1;
        this.intake2 = intake2;
    }

    public void startIntake() {
        // Have to check this
        intake1.setPower(-1);
        intake2.setPower(1);
    }

    public void reverseIntake() {
        intake1.setPower(1);
        intake2.setPower(-1);
    }

    public void stop() {
        intake1.setPower(0);
        intake2.setPower(0);
    }
}
