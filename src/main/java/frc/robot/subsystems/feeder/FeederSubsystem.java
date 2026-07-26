package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import  edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private TalonFX feederMotor;
    private TalonFXConfiguration feederMotorConfig;
    private TalonFXS kickerMotor;
    private TalonFXSConfiguration kickerMotorConfig;

    public FeederSubsystem() {
        // Construct your motors
        feederMotor = new TalonFX(19);
        kickerMotor = new TalonFXS(15);
        feederMotorConfig = new TalonFXConfiguration();
        kickerMotorConfig = new TalonFXSConfiguration();

        // Setup configs
        feederMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        kickerMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        kickerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kickerMotorConfig.Slot0.kP = 1.0;
        kickerMotorConfig.Slot0.kI = 0.0;
        kickerMotorConfig.Slot0.kD = 0.0;
        kickerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kickerMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        kickerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kickerMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;

        // Apply configs
        feederMotor.getConfigurator().apply(feederMotorConfig);
        kickerMotor.getConfigurator().apply(kickerMotorConfig);

        kickerMotor.setControl(new Follower(feederMotor.getDeviceID(), MotorAlignmentValue.Aligned));
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
