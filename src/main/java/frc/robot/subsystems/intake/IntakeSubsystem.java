// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX deployMotor;
  private final CANcoder deployCancoder;

  private final DigitalInput forwardDeployLimit;
  private final DigitalInput reverseDeployLimit;

  private final TalonFXConfiguration intakeConfig;
  private final TalonFXConfiguration deployConfig;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT, CANBus.roboRIO());
    deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_PORT, CANBus.roboRIO());
    deployCancoder = new CANcoder(IntakeConstants.DEPLOY_ENCODER_PORT, CANBus.roboRIO());

    intakeConfig = new TalonFXConfiguration();
    deployConfig = new TalonFXConfiguration();

    forwardDeployLimit = new DigitalInput(IntakeConstants.FORWARD_LIMIT_SWITCH_DIGITAL_PORT);
    reverseDeployLimit = new DigitalInput(IntakeConstants.REVERSE_LIMIT_SWITCH_DIGITAL_PORT);

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    intakeConfig.Feedback.RotorToSensorRatio = IntakeConstants.INTAKE_MOTOR_GEAR_RATIO;

    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    deployConfig.Feedback.FeedbackRemoteSensorID = deployCancoder.getDeviceID();   
    deployConfig.Feedback.RotorToSensorRatio = IntakeConstants.DEPLOY_MOTOR_GEAR_RATIO; 
    deployConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = forwardDeployLimit.getChannel();
    deployConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = reverseDeployLimit.getChannel();
    deployConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    deployConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    deployConfig.Slot0.kP = IntakeConstants.DEPLOY_KP;
    deployConfig.Slot0.kS = IntakeConstants.DEPLOY_KS;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param position Rotation2d of the position of the intake
   */
  public void setDeployPosition(Rotation2d position) {
    
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    deployMotor.setControl(m_request.withPosition(position.getRotations())
      .withLimitForwardMotion(true)
      .withLimitReverseMotion(true));
  }

  /**
   * @param velocity Velocity of the intake in rotations per second
   */
  public void runIntake(double velocity) {

    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    intakeMotor.setControl(m_request.withVelocity(velocity));
  }
}
