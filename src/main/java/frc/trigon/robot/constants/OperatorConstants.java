package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.trigon.hardware.misc.KeyboardController;
import org.trigon.hardware.misc.XboxController;

public class OperatorConstants {
    private static final int
            DRIVER_CONTROLLER_PORT = 0;
    private static final int DRIVER_CONTROLLER_EXPONENT = 1;
    private static final double DRIVER_CONTROLLER_DEADBAND = 0;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, DRIVER_CONTROLLER_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final double
            POV_DIVIDER = 2,
            STICKS_SPEED_DIVIDER = 1;

    public static final Trigger
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            TOGGLE_BRAKE_TRIGGER = OPERATOR_CONTROLLER.g().or(RobotController::getUserButton),
            TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER = DRIVER_CONTROLLER.b(),
            DRIVE_FROM_DPAD_TRIGGER = new Trigger(() -> DRIVER_CONTROLLER.getPov() != -1),
            ALIGN_TO_SPEAKER_TRIGGER = DRIVER_CONTROLLER.a(),
            ALIGN_TO_RIGHT_STAGE = OPERATOR_CONTROLLER.j(),
            ALIGN_TO_LEFT_STAGE = OPERATOR_CONTROLLER.h(),
            ALIGN_TO_MIDDLE_STAGE = OPERATOR_CONTROLLER.u(),
            TURN_AUTONOMOUS_NOTE_ALIGNING_ON_TRIGGER = OPERATOR_CONTROLLER.o(),
            TURN_AUTONOMOUS_NOTE_ALIGNING_OFF_TRIGGER = OPERATOR_CONTROLLER.p(),
            CLIMB_TRIGGER = OPERATOR_CONTROLLER.c(),
            OVERRIDE_IS_CLIMBING_TRIGGER = OPERATOR_CONTROLLER.i(),
            MOVE_CLIMBER_DOWN_MANUALLY_TRIGGER = OPERATOR_CONTROLLER.f1(),
            MOVE_CLIMBER_UP_MANUALLY_TRIGGER = OPERATOR_CONTROLLER.f2(),
            CONTINUE_TRIGGER = DRIVER_CONTROLLER.leftBumper().or(OPERATOR_CONTROLLER.k()),
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down(),
            COLLECT_NOTE_TRIGGER = DRIVER_CONTROLLER.leftTrigger(),
            EJECT_NOTE_TRIGGER = OPERATOR_CONTROLLER.e(),
            SPEAKER_SHOT_TRIGGER = DRIVER_CONTROLLER.rightBumper().or(OPERATOR_CONTROLLER.s()),
            CLOSE_SPEAKER_SHOT_TRIGGER = OPERATOR_CONTROLLER.x(),
            WARM_SPEAKER_SHOT_TRIGGER = OPERATOR_CONTROLLER.w().and(SPEAKER_SHOT_TRIGGER.negate()),
            DELIVERY_TRIGGER = OPERATOR_CONTROLLER.d(),
            MANUAL_LOW_DELIVERY_TRIGGER = OPERATOR_CONTROLLER.m(),
            AMP_TRIGGER = OPERATOR_CONTROLLER.a(),
            AUTONOMOUS_AMP_TRIGGER = OPERATOR_CONTROLLER.z(),
            ALIGN_TO_AMP_TRIGGER = DRIVER_CONTROLLER.x().and(OperatorConstants.AMP_TRIGGER.or(OperatorConstants.AUTONOMOUS_AMP_TRIGGER).negate());
}