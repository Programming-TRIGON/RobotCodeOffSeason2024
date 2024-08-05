package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class IntakeCommands {

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return new FunctionalCommand(
                () -> {
                },
                () -> RobotContainer.INTAKE.setTargetVoltage(targetState.voltage),
                (interrupted) -> RobotContainer.INTAKE.stop(),
                RobotContainer.INTAKE::hasNote,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetVoltageCommand(double collectorVoltage) {
        return new FunctionalCommand(
                () -> {
                },
                () -> RobotContainer.INTAKE.setTargetVoltage(collectorVoltage),
                (interrupted) -> RobotContainer.INTAKE.stop(),
                () -> false,
                RobotContainer.INTAKE
        );
    }

    public static Command getStopIntakeCommand() {
        return new StartEndCommand(
                RobotContainer.INTAKE::stop,
                () -> {
                },
                RobotContainer.INTAKE
        );
    }
}
