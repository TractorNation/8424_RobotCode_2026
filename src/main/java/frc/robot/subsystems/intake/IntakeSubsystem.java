package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeMotor;
    private TalonFXConfiguration intakeMotorConfig;

    public IntakeSubsystem() {
        // Construct your motors
        intakeMotor = new TalonFX(0);

        // Setup configs
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Apply configs
        intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    // Sets voltage for intake motor
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
