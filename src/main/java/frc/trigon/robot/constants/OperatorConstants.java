package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.trigon.hardware.misc.KeyboardController;
import org.trigon.hardware.misc.XboxController;

public class OperatorConstants {
    private static final int
            DRIVER_CONTROLLER_PORT = 0;
    private static final int DRIVER_CONTROLLER_EXPONENT = 1;
    private static final double DRIVER_CONTROLLER_DEADBAND = 0.1;
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
            FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.right(),
            BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.left(),
            FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.up(),
            BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = OPERATOR_CONTROLLER.down(),
            CONTINUE_TRIGGER = DRIVER_CONTROLLER.leftBumper().or(OPERATOR_CONTROLLER.k()),
            EJECT_NOTE_TRIGGER = OPERATOR_CONTROLLER.e(),
            SPEAKER_SHOT_TRIGGER = DRIVER_CONTROLLER.rightBumper().or(OPERATOR_CONTROLLER.s()),
            CLOSE_SPEAKER_SHOT_TRIGGER = OPERATOR_CONTROLLER.x(),
            WARM_SPEAKER_SHOT_TRIGGER = OPERATOR_CONTROLLER.w(),
            DELIVERY_TRIGGER = OPERATOR_CONTROLLER.d(),
            AMP_TRIGGER = DRIVER_CONTROLLER.x(),
            AUTONOMOUS_AMP_TRIGGER = OPERATOR_CONTROLLER.z();
}