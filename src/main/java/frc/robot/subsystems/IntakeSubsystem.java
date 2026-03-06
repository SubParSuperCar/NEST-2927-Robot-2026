package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(20);
    private final TalonFX deployMotor = new TalonFX(99); // ← placeholder CAN ID

    private final DutyCycleOut motorRequest = new DutyCycleOut(0);

    public void intakeIn() {
        intakeMotor.setControl(motorRequest.withOutput(0.75));
    }

    public void intakeOut() {
        intakeMotor.setControl(motorRequest.withOutput(-0.75));
    }

    public void stop() {
        intakeMotor.setControl(motorRequest.withOutput(0));
    }

    // --- Deploy motor methods ---
    public void deployOn() {
        deployMotor.setControl(motorRequest.withOutput(0.75));
    }

    public void deployOff() {
        deployMotor.setControl(motorRequest.withOutput(0));
    }
}
