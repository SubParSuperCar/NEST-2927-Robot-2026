package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // --- Motor declarations ---
    // Add more TalonFX objects here if you have multiple intake motors
    private final TalonFX intakeMotor1 = new TalonFX(20);
    
    // Control request (open-loop percent output, -1.0 to 1.0)
    private final DutyCycleOut motorRequest = new DutyCycleOut(0);

    // --- Intake In ---
    public void intakeIn() {
        intakeMotor1.setControl(motorRequest.withOutput(0.75)); // 75% forward
    }
    
    // --- Intake Out (eject) ---
    public void intakeOut() {
        intakeMotor1.setControl(motorRequest.withOutput(-0.75)); // 75% reverse
    }

    // --- Stop ---
    public void stop() {
        intakeMotor1.setControl(motorRequest.withOutput(0));
    }
}
