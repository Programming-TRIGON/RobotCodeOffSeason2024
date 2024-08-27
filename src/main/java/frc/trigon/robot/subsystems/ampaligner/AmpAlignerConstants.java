package frc.trigon.robot.subsystems.ampaligner;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;

public class AmpAlignerConstants {
    private static final int MOTOR_ID = 13;
    private static final String MOTOR_NAME = "AmpAlignerMotor";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME);

    private static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final ForwardLimitTypeValue FORWARD_LIMIT_TYPE_VALUE = ForwardLimitTypeValue.NormallyOpen;
    private static final ForwardLimitSourceValue FORWARD_LIMIT_SOURCE_VALUE = ForwardLimitSourceValue.LimitSwitchPin;
    private static final double
            P = RobotHardwareStats.isSimulation() ? 50 : 0,
            I = RobotHardwareStats.isSimulation() ? 0 : 0,
            D = RobotHardwareStats.isSimulation() ? 0 : 0,
            KS = RobotHardwareStats.isSimulation() ? 0.055582 : 0,
            KV = RobotHardwareStats.isSimulation() ? 6 : 0,
            KA = RobotHardwareStats.isSimulation() ? 0.12886 : 0;
    static final double KG = RobotHardwareStats.isSimulation() ? 0.11971 : 0;
    private static final double
            MOTION_MAGIC_ACCELERATION = RobotHardwareStats.isSimulation() ? 6 : 0,
            MOTION_MAGIC_CRUISE_VELOCITY = RobotHardwareStats.isSimulation() ? 6 : 0;
    private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
    private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE = StaticFeedforwardSignValue.UseVelocitySign;
    private static final double GEAR_RATIO = 52;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double AMP_ALIGNER_MASS_KILOGRAMS = 1.1;
    public static final double AMP_ALIGNER_LENGTH_METERS = 0.52;
    private static final Rotation2d
            AMP_ALIGNER_MINIMUM_ANGLE = Rotation2d.fromDegrees(173.7 - 156),
            AMP_ALIGNER_MAXIMUM_ANGLE = Rotation2d.fromDegrees(173.7);
    private static final Rotation2d REVERSE_SOFT_LIMIT_THRESHOLD = Rotation2d.fromDegrees(173.7 - 156);
    private static final boolean SIMULATE_GRAVITY = true;
    private static final SingleJointedArmSimulation SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            AMP_ALIGNER_LENGTH_METERS,
            AMP_ALIGNER_MASS_KILOGRAMS,
            AMP_ALIGNER_MINIMUM_ANGLE,
            AMP_ALIGNER_MAXIMUM_ANGLE,
            SIMULATE_GRAVITY
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(2).per(Units.Second.of(1)),
            Units.Volts.of(4),
            Units.Second.of(1000)
    );

    public static final Transform3d PITCHER_TO_AMP_ALIGNER = new Transform3d(-0.4838, 0, 0.1472, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(24), 0));

    static final Rotation2d READY_FOR_DEFAULT_PITCHER_MOVEMENT_ANGLE = Rotation2d.fromDegrees(80);
    static final Rotation2d LIMIT_SWITCH_PRESSED_ANGLE = Rotation2d.fromDegrees(173.7);
    static final double LIMIT_SWITCH_DEBOUNCE_TIME_SECONDS = 0.1;
    static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.3);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;

        config.HardwareLimitSwitch.ForwardLimitType = FORWARD_LIMIT_TYPE_VALUE;
        config.HardwareLimitSwitch.ForwardLimitSource = FORWARD_LIMIT_SOURCE_VALUE;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        if (RobotHardwareStats.isSimulation())
            config.Slot0.kG = KG;
        config.Slot0.GravityType = GRAVITY_TYPE_VALUE;
        config.Slot0.StaticFeedforwardSign = STATIC_FEEDFORWARD_SIGN_VALUE;

        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT_THRESHOLD.getRotations();

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MOTOR.registerSignal(TalonFXSignal.FORWARD_LIMIT, 100);
        MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    public enum AmpAlignerState {
        OPEN(Rotation2d.fromDegrees(173.7 - 156)),
        CLOSE(Rotation2d.fromDegrees(173.7));

        public final Rotation2d targetAngle;

        AmpAlignerState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }
    }
}
