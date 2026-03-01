package frc.robot.subsystems.intake;

import java.util.logging.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFXS rollerMotor;
    private TalonFXS armMotorA;
    private TalonFXS armMotorB;
    private TalonFXSConfiguration rollerMotorConfig;
    private TalonFXSConfiguration armMotorConfig;

    private StatusSignal<Angle> armMotorPosition;

    double maxArmPosition;

    public IntakeSubsystem(double maxArmPosition) {
        // Construct your motors
        rollerMotor = new TalonFXS(16, "rio");
        armMotorA = new TalonFXS(17, "rio");
        armMotorB = new TalonFXS(18, "rio");

        rollerMotorConfig = new TalonFXSConfiguration();
        armMotorConfig = new TalonFXSConfiguration();

        // Setup configs
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        armMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfig.Slot0.kP = 0.7;
        armMotorConfig.Slot0.kI = 0.0;
        armMotorConfig.Slot0.kD = 0.0;
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        armMotorB.setControl(new Follower(armMotorA.getDeviceID(), MotorAlignmentValue.Opposed));

        armMotorPosition = armMotorA.getPosition();

        // Apply configs
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
        armMotorA.getConfigurator().apply(armMotorConfig);
        armMotorB.getConfigurator().apply(armMotorConfig);

        this.maxArmPosition = maxArmPosition;
    }

    // Sets voltage for intake motor
    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    public void setArmPosition(double position) {
        armMotorA.setControl(new PositionVoltage(position));
    }

    @Override
    public void periodic() {
        armMotorPosition.refresh();
        SmartDashboard.putNumber("ArmMotorPosition", armMotorPosition.getValueAsDouble());
    }

    public void incrementArmPos(double value){
        double armPos = armMotorPosition.getValueAsDouble();
        
        double incremented = armPos + value;
        if(incremented <= maxArmPosition && incremented >= 0){
            setArmPosition(incremented);
        } else {
            return;
        }
    }
}
