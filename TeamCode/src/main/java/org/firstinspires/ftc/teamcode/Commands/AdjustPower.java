package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class AdjustPower extends CommandBase {
    ShooterSubsystem shooter;
    VisionSubsystem vision;
    VoltageSensor battery;
    int id;
    public AdjustPower(ShooterSubsystem shooter, VisionSubsystem vision, int id, VoltageSensor battery) {
        this.shooter = shooter;
        this.vision = vision;
        this.battery = battery;
        this.id = id;
    }

    @Override
    public void execute() {
        if (vision.getApriltagTx(id) != -1000) {
            double tx = vision.getApriltagTx(id);

            double d = (CameraConstants.apriltagHeight-CameraConstants.height) /
                    Math.tan(CameraConstants.pitch + Math.toRadians(tx));
            double power = ShooterConstants.startingPower
                    - (battery.getVoltage()-12)*ShooterConstants.batteryModifier
                    + d*ShooterConstants.distanceModifier;
            if (d > 300) {
                power -= 0.02;
            }
            shooter.setPower(power);
        }
    }
}
