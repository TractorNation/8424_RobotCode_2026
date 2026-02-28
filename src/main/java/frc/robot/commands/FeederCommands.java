package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class FeederCommands {

    public static Command runFeeder(FeederSubsystem feeder, double speed) {
        return Commands.runOnce(() -> feeder.setFeederVoltage(speed), feeder);
    }

    public static Command stopFeeder(FeederSubsystem feeder) {
        return Commands.runOnce(() -> feeder.setFeederVoltage(0), feeder);
    }
}
