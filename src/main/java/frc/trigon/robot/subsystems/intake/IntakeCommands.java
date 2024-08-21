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
        if (targetState == IntakeConstants.IntakeState.COLLECTING) {
            return getCollectionCommand();
        }
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

    private static Command getStopIntakeCommand() {
        return new InstantCommand(
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    private static Command getCollectionCommand() {
        return new SequentialCommandGroup(
                new FunctionalCommand(
                        () -> RobotContainer.INTAKE.setTargetState(IntakeConstants.IntakeState.COLLECTING),
                        () -> {
                        },
                        (interrupted) -> {
                            RobotContainer.INTAKE.sendStaticBrakeRequest();
                            RobotContainer.INTAKE.indicateCollection();
                        },
                        RobotContainer.INTAKE::hasNote,
                        RobotContainer.INTAKE
                ),
                getWaitForNoteToStopCommand().andThen(getStopIntakeCommand())
        );
    }


    private static Command getWaitForNoteToStopCommand() {
        return new WaitCommand(IntakeConstants.NOTE_STOPPING_SECONDS);
    }
}