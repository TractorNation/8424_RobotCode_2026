package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX rollerMotor;
    private TalonFX armMotorA;
    private TalonFX armMotorB;
    private TalonFXConfiguration rollerMotorConfig;
    private TalonFXConfiguration armMotorConfig;

    public IntakeSubsystem() {
        // Construct your motors
        rollerMotor = new TalonFX(0);
        armMotorA = new TalonFX(1);
        armMotorB = new TalonFX(2);

        // Setup configs
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        armMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfig.Slot0.kP = 0.1;
        armMotorConfig.Slot0.kI = 0.0;
        armMotorConfig.Slot0.kD = 0.0;
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armMotorB.setControl(new Follower(armMotorA.getDeviceID(), MotorAlignmentValue.Opposed));

        // Apply configs
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
        armMotorA.getConfigurator().apply(armMotorConfig);
        armMotorB.getConfigurator().apply(armMotorConfig);
    }

    // Sets voltage for intake motor
    public void setIntakeVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    public void setArmPosition(double position) {
        armMotorA.setControl(new PositionVoltage(position));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
