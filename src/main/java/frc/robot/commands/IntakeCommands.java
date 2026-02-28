package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommands {

    public static Command extendArm(IntakeSubsystem intake, double rotations) {
        return Commands.runOnce(() -> intake.setArmPosition(rotations), intake);
    }

    public static Command pullArm(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.setArmPosition(0), intake);
    }

    public static Command runRoller(IntakeSubsystem intake, double speed) {
        return Commands.runOnce(() -> intake.setRollerVoltage(speed), intake);
    }

    public static Command stopRoller(IntakeSubsystem intake) {
        return Commands.runOnce(() -> intake.setRollerVoltage(0), intake);
    }
}
