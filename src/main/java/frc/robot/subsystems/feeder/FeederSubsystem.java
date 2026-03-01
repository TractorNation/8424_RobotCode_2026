package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import  edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private TalonFXS feederMotor;
    private TalonFXSConfiguration feederMotorConfig;

    public FeederSubsystem() {
        // Construct your motors
        feederMotor = new TalonFXS(19, "rio");

        feederMotorConfig = new TalonFXSConfiguration();

        // Setup configs
        feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // Apply configs
        feederMotor.getConfigurator().apply(feederMotorConfig);
    }

    // Sets voltage for intake motor
    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
