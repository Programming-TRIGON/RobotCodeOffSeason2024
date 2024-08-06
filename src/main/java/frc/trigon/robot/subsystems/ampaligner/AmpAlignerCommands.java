package frc.trigon.robot.subsystems.ampaligner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.trigon.robot.RobotContainer;

public class AmpAlignerCommands {
    public static Command getOpenAmpAlignerCommand() {
        return new FunctionalCommand(
                RobotContainer.AMP_ALIGNER::openAmpAligner,
                () -> {
                },
                (interrupted) -> RobotContainer.AMP_ALIGNER.stop(),
                RobotContainer.AMP_ALIGNER::isForwardLimitSwitchPressed,
                RobotContainer.AMP_ALIGNER
        );
    }

    public static Command getCloseAmpAlignerCommand() {
        return new FunctionalCommand(
                RobotContainer.AMP_ALIGNER::closeAmpAligner,
                () -> {
                },
                (interrupted) -> RobotContainer.AMP_ALIGNER.stop(),
                RobotContainer.AMP_ALIGNER::isBackwardLimitSwitchPressed,
                RobotContainer.AMP_ALIGNER
        );
    }
}
