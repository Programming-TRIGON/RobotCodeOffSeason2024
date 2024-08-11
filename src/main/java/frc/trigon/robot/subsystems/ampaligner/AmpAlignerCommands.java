package frc.trigon.robot.subsystems.ampaligner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

public class AmpAlignerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> AmpAlignerCommands.getSetTargetAngleCommand(Rotation2d.fromDegrees(targetAngleDegrees)),
                false,
                "Debugging/TargetDebuggingAmpAlignerDegrees"
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new FunctionalCommand(
                () -> RobotContainer.AMP_ALIGNER.setTargetAngle(targetAngle),
                RobotContainer.AMP_ALIGNER::setMotorFeedForwardFromCurrentAngle,
                (interrupted) -> RobotContainer.AMP_ALIGNER.stop(),
                () -> false,
                RobotContainer.AMP_ALIGNER
        );
    }

    public static Command getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState targetState) {
        return new FunctionalCommand(
                () -> RobotContainer.AMP_ALIGNER.setTargetState(targetState),
                RobotContainer.AMP_ALIGNER::setMotorFeedForwardFromCurrentAngle,
                (interrupted) -> RobotContainer.AMP_ALIGNER.stop(),
                () -> false,
                RobotContainer.AMP_ALIGNER
        );
    }
}
