// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.LEDAutoSetupCommand;
import frc.trigon.robot.commands.factories.AmpCommands;
import frc.trigon.robot.commands.factories.AutonomousCommands;
import frc.trigon.robot.commands.factories.GeneralCommands;
import frc.trigon.robot.commands.factories.ShootingCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.ampaligner.AmpAligner;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerCommands;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerConstants;
import frc.trigon.robot.subsystems.climber.Climber;
import frc.trigon.robot.subsystems.intake.Intake;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.pitcher.Pitcher;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.Shooter;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            CameraConstants.FRONT_TAG_CAMERA,
            CameraConstants.REAR_TAG_CAMERA
    );
    public static final Swerve SWERVE = new Swerve();
    public static final Intake INTAKE = new Intake();
    public static final Climber CLIMBER = new Climber();
    public static final Pitcher PITCHER = new Pitcher();
    public static final Shooter SHOOTER = new Shooter();
    public static final AmpAligner AMP_ALIGNER = new AmpAligner();
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
        INTAKE.setDefaultCommand(CommandConstants.DEFAULT_INTAKE_COMMAND);
        CLIMBER.setDefaultCommand(CommandConstants.DEFAULT_CLIMBER_COMMAND);
        AMP_ALIGNER.setDefaultCommand(CommandConstants.DEFAULT_AMP_ALIGNER_COMMAND);
        PITCHER.setDefaultCommand(GeneralCommands.getDefaultPitcherCommand());
        SHOOTER.setDefaultCommand(ShooterCommands.getStopCommand());
        LEDStrip.setDefaultCommandForAllLEDS(CommandConstants.DEFAULT_LEDS_COMMAND);
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.SET_GYRO_HEADING_TO_SOLVE_PNP_HEADING_TRIGGER.whileTrue(CommandConstants.SET_GYRO_HEADING_TO_SOLVE_PNP_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER.onTrue(GeneralCommands.getToggleFieldAndSelfRelativeDriveCommand());
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(GeneralCommands.getToggleBrakeCommand());
        OperatorConstants.ALIGN_TO_RIGHT_STAGE.whileTrue(CommandConstants.ALIGN_TO_RIGHT_STAGE_COMMAND);
        OperatorConstants.ALIGN_TO_LEFT_STAGE.whileTrue(CommandConstants.ALIGN_TO_LEFT_STAGE_COMMAND);
        OperatorConstants.ALIGN_TO_MIDDLE_STAGE.whileTrue(CommandConstants.ALIGN_TO_MIDDLE_STAGE_COMMAND);

        OperatorConstants.EJECT_NOTE_TRIGGER.whileTrue(CommandConstants.EJECT_COMMAND);
        OperatorConstants.HIGH_EJECT_NOTE_TRIGGER.whileTrue(GeneralCommands.getHighEjectNoteCommand());
        OperatorConstants.COLLECT_NOTE_TRIGGER.whileTrue(GeneralCommands.getNoteCollectionCommand());
        OperatorConstants.TURN_AUTONOMOUS_NOTE_ALIGNING_ON_TRIGGER.onTrue(CommandConstants.TURN_AUTONOMOUS_NOTE_ALIGNING_ON_COMMAND);
        OperatorConstants.TURN_AUTONOMOUS_NOTE_ALIGNING_OFF_TRIGGER.onTrue(CommandConstants.TURN_AUTONOMOUS_NOTE_ALIGNING_OFF_COMMAND);

        OperatorConstants.LED_AUTO_SETUP_TRIGGER.whileTrue(new LEDAutoSetupCommand(() -> autoChooser.get().getName()));

        OperatorConstants.CLIMB_TRIGGER.whileTrue(GeneralCommands.getClimbCommand());
        OperatorConstants.MOVE_CLIMBER_DOWN_MANUALLY_TRIGGER.whileTrue(CommandConstants.MOVE_CLIMBER_DOWN_MANUALLY_COMMAND);
        OperatorConstants.MOVE_CLIMBER_UP_MANUALLY_TRIGGER.whileTrue(CommandConstants.MOVE_CLIMBER_UP_MANUALLY_COMMAND);
        OperatorConstants.MOVE_RIGHT_CLIMBER_DOWN_MANUALLY.whileTrue(CommandConstants.MOVE_RIGHT_CLIMBER_DOWN_MANUALLY_COMMAND);
        OperatorConstants.MOVE_RIGHT_CLIMBER_UP_MANUALLY.whileTrue(CommandConstants.MOVE_RIGHT_CLIMBER_UP_MANUALLY_COMMAND);
        OperatorConstants.MOVE_LEFT_CLIMBER_DOWN_MANUALLY.whileTrue(CommandConstants.MOVE_LEFT_CLIMBER_DOWN_MANUALLY_COMMAND);
        OperatorConstants.MOVE_LEFT_CLIMBER_UP_MANUALLY.whileTrue(CommandConstants.MOVE_LEFT_CLIMBER_UP_MANUALLY_COMMAND);
        OperatorConstants.OVERRIDE_IS_CLIMBING_TRIGGER.onTrue(CommandConstants.OVERRIDE_IS_CLIMBING_COMMAND);

        OperatorConstants.SPEAKER_SHOT_TRIGGER.whileTrue(CommandConstants.SHOOT_AT_SPEAKER_COMMAND);
        OperatorConstants.CLOSE_SPEAKER_SHOT_TRIGGER.whileTrue(ShootingCommands.getCloseSpeakerShotCommand());
        OperatorConstants.WARM_SPEAKER_SHOT_TRIGGER.whileTrue(ShootingCommands.getWarmSpeakerShotCommand());
        OperatorConstants.DELIVERY_TRIGGER.whileTrue(CommandConstants.DELIVERY_COMMAND);
        OperatorConstants.MANUAL_LOW_DELIVERY_TRIGGER.whileTrue(ShootingCommands.getManualLowDeliveryCommand());

        OperatorConstants.AMP_TRIGGER.whileTrue(AmpCommands.getScoreInAmpCommand(true));
        OperatorConstants.AMP_WITHOUT_ALIGN_TRIGGER.whileTrue(AmpCommands.getScoreInAmpCommand(false));
        OperatorConstants.AUTONOMOUS_AMP_TRIGGER.whileTrue(AmpCommands.getAutonomousScoreInAmpCommand());
        OperatorConstants.REALIGN_AMP_ALIGNER_TRIGGER.whileTrue(AmpAlignerCommands.getSetPositionCommand(AmpAlignerConstants.REALIGN_AMP_ALIGNER_ANGLE));
        OperatorConstants.BLOCK_TRIGGER.whileTrue(AmpCommands.getBlockCommand());

        OperatorConstants.RESET_POSE_TO_AUTO_POSE_TRIGGER.onTrue(AutonomousCommands.getResetPoseToAutoPoseCommand(() -> autoChooser.get().getName()));

        OperatorConstants.OPERATOR_CONTROLLER.t().whileTrue(PitcherCommands.getDebuggingCommand());
    }

    private void configureSysIdBindings(MotorSubsystem subsystem) {
        OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        subsystem.setDefaultCommand(edu.wpi.first.wpilibj2.command.Commands.idle(subsystem));
    }

    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser("FrontRow4NoteAuto"));
    }
}