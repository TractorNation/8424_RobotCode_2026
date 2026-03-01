package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterMode;

public class ShooterCommands {

    public static Command updateShooterState(ShooterSubsystem shooter, ShooterMode mode) {
        return Commands.runOnce(() -> {
            switch (mode) {
                case LOW:
                    shooter.setShooterVelocity(50);
                    //shooter.setHoodPosition(0.1);
                    break;
                case MID:
                    shooter.setShooterVelocity(75);
                    //shooter.setHoodPosition(0.2);
                    break;
                case HIGH:
                    shooter.setShooterVelocity(100);
                    //shooter.setHoodPosition(0.3);
                    break;
            }       
        }, shooter);
    }

    public static Command stopShooter(ShooterSubsystem shooter) {
        return Commands.runOnce(() -> shooter.setShooterVelocity(0), shooter);
    }
}
