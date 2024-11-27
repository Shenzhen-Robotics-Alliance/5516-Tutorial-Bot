package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeFalcon1, intakeFalcon2;
    public Intake() {
        this.intakeFalcon1 = new TalonFX(13);
        intakeFalcon1.setInverted(true);
        this.intakeFalcon2 = new TalonFX(15);
        intakeFalcon2.setInverted(false);
    }

    public Command runIntake() {
        return run(() -> intakeFalcon1.set(0.6));
    }

    public Command runInverseALittle() {
        return run(() -> intakeFalcon1.set(-0.2)).withTimeout(0.2);
    }
}
