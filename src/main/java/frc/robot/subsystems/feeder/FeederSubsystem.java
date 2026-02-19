package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import  edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private TalonFX feederMotor;
    private TalonFXConfiguration feederMotorConfig;

    public FeederSubsystem() {
        // Construct your motors
        feederMotor = new TalonFX(0);

        // Setup configs
        feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Apply configs
        feederMotor.getConfigurator().apply(feederMotorConfig);
    }

    // Sets voltage for intake motor
    public void setFeederVoltage(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
