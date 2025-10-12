package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem extends SubsystemBase {
    // I honestly have no idea what motors and stuff should go in here
    public DcMotor intake;
    public IntakeSubsystem (DcMotor intake) {
        this.intake = intake;
    }
}
