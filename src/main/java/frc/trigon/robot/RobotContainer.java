// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.ampaligner.AmpAligner;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerCommands;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerConstants;
import frc.trigon.robot.subsystems.pitcher.Pitcher;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import frc.trigon.robot.subsystems.shooter.Shooter;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final Swerve SWERVE = new Swerve();
    public static final Pitcher PITCHER = new Pitcher();
    public static final Shooter SHOOTER = new Shooter();
    public static final AmpAligner AMP_ALIGNER = new AmpAligner();
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator();
    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        buildAutoChooser();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
    }

    private void bindDefaultCommands() {
        SWERVE.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);
        AMP_ALIGNER.setDefaultCommand(AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.CLOSE));
        PITCHER.setDefaultCommand(PitcherCommands.getSetTargetPitchCommand(PitcherConstants.DEFAULT_PITCH));
        SHOOTER.setDefaultCommand(ShooterCommands.getStopCommand());
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER.onTrue(Commands.getToggleFieldAndSelfRelativeDriveCommand());
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(Commands.getToggleBrakeCommand());

        OperatorConstants.OPERATOR_CONTROLLER.s().whileTrue(ShooterCommands.getSetTargetVelocity(50, 50));
        OperatorConstants.OPERATOR_CONTROLLER.p().whileTrue(PitcherCommands.getSetTargetPitchCommand(Rotation2d.fromDegrees(50)));
        OperatorConstants.OPERATOR_CONTROLLER.t().whileTrue(AmpAlignerCommands.getSetTargetAngleCommand(Rotation2d.fromDegrees(90)));
        OperatorConstants.OPERATOR_CONTROLLER.y().whileTrue(AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.OPEN));

        OperatorConstants.OPERATOR_CONTROLLER.z().whileTrue(AMP_ALIGNER.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.OPERATOR_CONTROLLER.x().whileTrue(AMP_ALIGNER.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.OPERATOR_CONTROLLER.c().whileTrue(AMP_ALIGNER.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.OPERATOR_CONTROLLER.v().whileTrue(AMP_ALIGNER.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
    }

    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }
}
