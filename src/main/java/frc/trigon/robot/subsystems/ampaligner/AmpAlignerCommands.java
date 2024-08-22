package frc.trigon.robot.subsystems.ampaligner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.NetworkTablesCommand;

public class AmpAlignerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> AmpAlignerCommands.getSetTargetAngleCommand(Rotation2d.fromDegrees(targetAngleDegrees)),
                true,
                "Debugging/TargetDebuggingAmpAlignerDegrees"
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new ExecuteEndCommand(
                () -> RobotContainer.AMP_ALIGNER.setTargetAngle(targetAngle),
                RobotContainer.AMP_ALIGNER::stop,
                RobotContainer.AMP_ALIGNER
        );
    }

    public static Command getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState targetState) {
        return new ExecuteEndCommand(
                () -> RobotContainer.AMP_ALIGNER.setTargetState(targetState),
                RobotContainer.AMP_ALIGNER::stop,
                RobotContainer.AMP_ALIGNER
        );
    }
}
