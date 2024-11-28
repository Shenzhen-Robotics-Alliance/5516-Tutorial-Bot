package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX shooter1, shooter2;

    public Shooter() {
        shooter1 = new TalonFX(14);
        shooter1.setInverted(true);

        shooter2 = new TalonFX(16);
        shooter2.setInverted(true);

        setDefaultCommand(runIdle());
    }

    public Command runReverse() {
        return runPercentPower(-0.2);
    }

    public Command runForward() {
        return runPercentPower(0.5);
    }

    public Command runIdle() {
        return runPercentPower(0);
    }

    public Command runPercentPower(double percentPower) {
        return run(() -> {
            shooter1.set(percentPower);
            shooter2.set(percentPower);
        });
    }
}
