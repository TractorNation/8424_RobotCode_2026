// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX shooterMotor;
  private TalonFXConfiguration shooterMotorConfig;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Construct your motors
    shooterMotor = new TalonFX(0);

    // Setup the configs
    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotorConfig.Slot0.kP = 0.1;
    shooterMotorConfig.Slot0.kI = 0.0;
    shooterMotorConfig.Slot0.kD = 0.0;
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    //secondShooterMotor.setControl(new Follower(0, false));

    // Apply the configs
    shooterMotor.getConfigurator().apply(shooterMotorConfig);
  }

/**
 * 
 * @param velocity The velocity of the shooter in rps (max 100)
 */
  public void setShooterVelocity(double velocity){
    shooterMotor.setControl(new VelocityVoltage(velocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
