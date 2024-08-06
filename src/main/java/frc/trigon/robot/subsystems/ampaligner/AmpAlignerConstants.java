package frc.trigon.robot.subsystems.ampaligner;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.mechanisms.SingleJointedArmMechanism2d;

import java.util.function.DoubleSupplier;

public class AmpAlignerConstants {
    private static final int
            MOTOR_ID = 0,
            FORWARD_LIMIT_SWITCH_ID = 0,
            BACKWARD_LIMIT_SWITCH_ID = 1;
    private static final String
            MOTOR_NAME = "AmpAlignerMotor",
            FORWARD_LIMIT_SWITCH_NAME = "AmpAlignerForwardLimitSwitch",
            BACKWARD_LIMIT_SWITCH_NAME = "AmpAlignerBackwardLimitSwitch";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME, RobotConstants.CANIVORE_NAME);
    static final SimpleSensor
            FORWARD_LIMIT_SWITCH = SimpleSensor.createDigitalSensor(FORWARD_LIMIT_SWITCH_ID, FORWARD_LIMIT_SWITCH_NAME),
            BACKWARD_LIMIT_SWITCH = SimpleSensor.createDigitalSensor(BACKWARD_LIMIT_SWITCH_ID, BACKWARD_LIMIT_SWITCH_NAME);
    private static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            P = RobotHardwareStats.isSimulation() ? 1 : 1,
            I = RobotHardwareStats.isSimulation() ? 0 : 0,
            D = RobotHardwareStats.isSimulation() ? 0 : 0,
            KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            KA = RobotHardwareStats.isSimulation() ? 0 : 0,
            KG = RobotHardwareStats.isSimulation() ? 0 : 0;
    private static final double GEAR_RATIO = 1;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double
            AMP_ALIGNER_LENGTH_METERS = 0.5,
            AMP_ALIGNER_MASS_KILOGRAMS = 0.5;
    private static final Rotation2d
            AMP_ALIGNER_MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            AMP_ALIGNER_MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final DoubleSupplier
            FORWARD_LIMIT_SWITCH_SUPPLIER = () -> 0,
            BACKWARD_LIMIT_SWITCH_SUPPLIER = () -> 0;
    private static final SingleJointedArmSimulation SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            AMP_ALIGNER_LENGTH_METERS,
            AMP_ALIGNER_MASS_KILOGRAMS,
            AMP_ALIGNER_MINIMUM_ANGLE,
            AMP_ALIGNER_MAXIMUM_ANGLE,
            true
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Second.of(1)),
            Units.Volts.of(2),
            Units.Second.of(1000)
    );

    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            "AmpAlignerMechanism",
            AMP_ALIGNER_LENGTH_METERS,
            new Color8Bit(Color.kYellow)
    );

    static {
        configureMotor();
        configureLimitSwitches();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kG = KG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.Feedback.RotorToSensorRatio = GEAR_RATIO;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureLimitSwitches() {
        FORWARD_LIMIT_SWITCH.setSimulationSupplier(FORWARD_LIMIT_SWITCH_SUPPLIER);
        BACKWARD_LIMIT_SWITCH.setSimulationSupplier(BACKWARD_LIMIT_SWITCH_SUPPLIER);
    }

    public enum AmpAlignerState {
        OPENING(5, AMP_ALIGNER_MAXIMUM_ANGLE),
        CLOSING(-5, AMP_ALIGNER_MINIMUM_ANGLE);

        public final double voltage;
        public final Rotation2d targetPosition;

        AmpAlignerState(double voltage, Rotation2d targetPosition) {
            this.voltage = voltage;
            this.targetPosition = targetPosition;
        }
    }
}
