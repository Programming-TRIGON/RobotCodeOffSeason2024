package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

public class IntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                IntakeCommands::getSetTargetVoltageCommand,
                false,
                "Debugging/TargetDebuggingIntakeVoltage"
        );
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        if (targetState == IntakeConstants.IntakeState.COLLECT)
            return getCollectionCommand();
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetState(targetState),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetVoltage(targetVoltage),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    private static Command getCollectionCommand() {
        return new SequentialCommandGroup(
                new FunctionalCommand(
                        () -> RobotContainer.INTAKE.setTargetState(IntakeConstants.IntakeState.COLLECT),
                        () -> {
                        },
                        (interrupted) -> {
                            if (!interrupted) {
                                RobotContainer.INTAKE.indicateCollection();
                                RobotContainer.INTAKE.sendStaticBrakeRequest();
                            }
                        },
                        RobotContainer.INTAKE::hasNote,
                        RobotContainer.INTAKE
                ),
                getCorrectNotePositionCommand().andThen(getStopIntakeCommand())
        );
    }

    private static Command getStopIntakeCommand() {
        return new InstantCommand(
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }


    private static Command getCorrectNotePositionCommand() {
        return getSetTargetStateCommand(IntakeConstants.IntakeState.CORRECT_NOTE_POSITION).withTimeout(IntakeConstants.CORRECT_NOTE_POSITION_TIMEOUT_SECONDS);
    }
}