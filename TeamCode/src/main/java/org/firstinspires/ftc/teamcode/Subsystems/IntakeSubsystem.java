package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    public DcMotor intake;
    public IntakeSubsystem (DcMotor intake) {
        this.intake = intake;
    }

    public void startIntake() {
        // Have to check this
        intake.setPower(IntakeConstants.forward);
    }

    public void reverseIntake() {
        intake.setPower(IntakeConstants.backward);
    }

    public void stop() {
        intake.setPower(0);
    }
}
