// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX shooterMotorA;
  private TalonFX shooterMotorB;
  //private TalonFXS kickerMotor;
  private TalonFXConfiguration shooterMotorConfig;
  //private TalonFXSConfiguration kickerMotorConfig;

  public enum ShooterMode {
    LOW, MID, HIGH
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Construct your motors
    shooterMotorA = new TalonFX(13);
    shooterMotorB = new TalonFX(14);
   // kickerMotor = new TalonFXS(15);

    shooterMotorConfig = new TalonFXConfiguration();
    //kickerMotorConfig = new TalonFXSConfiguration();

    // Setup the configs
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterMotorConfig.Slot0.kP = 1.0;
    shooterMotorConfig.Slot0.kI = 0.0;
    shooterMotorConfig.Slot0.kD = 0.0;
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;

    //kickerMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    //kickerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //kickerMotorConfig.Slot0.kP = 1.0;
    //kickerMotorConfig.Slot0.kI = 0.0;
    //kickerMotorConfig.Slot0.kD = 0.0;
    //kickerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //kickerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    //kickerMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;


    // Apply the configs
    shooterMotorA.getConfigurator().apply(shooterMotorConfig);
    shooterMotorB.getConfigurator().apply(shooterMotorConfig);
    //kickerMotor.getConfigurator().apply(kickerMotorConfig);

    shooterMotorB.setControl(new Follower(shooterMotorA.getDeviceID(), MotorAlignmentValue.Opposed));
    //kickerMotor.setControl(new Follower(shooterMotorA.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  /**
   * 
   * @param velocity The velocity of the shooter in rps (max 100)
   */
  public void setShooterVelocity(double velocity) {
    shooterMotorA.setControl(new VelocityVoltage(velocity));
  }

  public void stopShooter() {
    shooterMotorA.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // SmartDashboard.putNumber("ShooterSpeedRPS", shooterMotorA.getVelocity().getValueAsDouble());
  }
}
