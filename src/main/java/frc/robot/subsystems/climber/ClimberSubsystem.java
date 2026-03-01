package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase { 
    private TalonFX climberMotor;
    private TalonFXConfiguration climberMotorConfig; 

    public ClimberSubsystem() {
        climberMotor = new TalonFX(19);

        climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        climberMotorConfig.Slot0.kP = 0.1;
        climberMotorConfig.Slot0.kI = 0.0;
        climberMotorConfig.Slot0.kD = 0.0;
        climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climberMotor.getConfigurator().apply(climberMotorConfig);
    }

    public void setClimberPosition(double position) {
        climberMotor.setControl(new PositionVoltage(position));
    }
    
    @Override
    public void periodic() {

    }

}