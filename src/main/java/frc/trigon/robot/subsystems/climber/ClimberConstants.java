package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;

public class ClimberConstants {
    private static final int
            RIGHT_MOTOR_ID = 14,
            LEFT_MOTOR_ID = 15;
    private static final String
            RIGHT_MOTOR_NAME = "RightClimberMotor",
            LEFT_MOTOR_NAME = "LeftClimberMotor";
    static final TalonFXMotor
            RIGHT_MOTOR = new TalonFXMotor(RIGHT_MOTOR_ID, RIGHT_MOTOR_NAME),
            LEFT_MOTOR = new TalonFXMotor(LEFT_MOTOR_ID, LEFT_MOTOR_NAME);

    private static final InvertedValue
            RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            LEFT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    static final boolean ENABLE_FOC = true;
    static final double
            MAX_GROUNDED_VELOCITY = RobotHardwareStats.isSimulation() ? 4 : 0,
            MAX_GROUNDED_ACCELERATION = RobotHardwareStats.isSimulation() ? 4 : 0,
            MAX_ON_CHAIN_VELOCITY = RobotHardwareStats.isSimulation() ? 4 : 0,
            MAX_ON_CHAIN_ACCELERATION = RobotHardwareStats.isSimulation() ? 4 : 0;
    private static final double //TODO: calibrate
            GROUNDED_P = RobotHardwareStats.isSimulation() ? 700 : 0,
            GROUNDED_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_KS = RobotHardwareStats.isSimulation() ? 0.0046109 : 0,
            GROUNDED_KV = RobotHardwareStats.isSimulation() ? 8.7858 : 0,
            GROUNDED_KA = RobotHardwareStats.isSimulation() ? 0.17776 : 0;
    static final double
            GROUNDED_A = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_B = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_C = RobotHardwareStats.isSimulation() ? 0 : 0;
    private static final double //TODO: calibrate
            ON_CHAIN_P = RobotHardwareStats.isSimulation() ? 700 : 0,
            ON_CHAIN_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_KS = RobotHardwareStats.isSimulation() ? 0.0046109 : 0,
            ON_CHAIN_KV = RobotHardwareStats.isSimulation() ? 8.7858 : 0,
            ON_CHAIN_KA = RobotHardwareStats.isSimulation() ? 0.17776 : 0;
    static final double
            ON_CHAIN_A = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_B = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_C = RobotHardwareStats.isSimulation() ? 0 : 0;
    static final int
            GROUNDED_SLOT = 0,
            ON_CHAIN_SLOT = 1;
    private static final double
            REVERSE_SOFT_LIMIT_POSITION_ROTATIONS = 0,
            FORWARD_SOFT_LIMIT_POSITION_ROTATIONS = 3.183;
    static final double GEAR_RATIO = 74.67;

    private static final int
            RIGHT_MOTOR_AMOUNT = 1,
            LEFT_MOTOR_AMOUNT = 1;
    private static final DCMotor
            RIGHT_GEARBOX = DCMotor.getFalcon500Foc(RIGHT_MOTOR_AMOUNT),
            LEFT_GEARBOX = DCMotor.getFalcon500Foc(LEFT_MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation
            RIGHT_MOTOR_SIMULATION = new SimpleMotorSimulation(RIGHT_GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA),
            LEFT_MOTOR_SIMULATION = new SimpleMotorSimulation(LEFT_GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA);

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Second.of(1)),
            Units.Volts.of(8),
            null,
            null
    );
    static final boolean SYSID_IS_ON_CHAIN = true;

    static final Translation3d
            RIGHT_CLIMBER_FIRST_JOINT_ORIGIN_POINT = new Translation3d(0.295, -254.5, 274.45),
            LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT = new Translation3d(0.295, 254.5, 274.45);
    static final double FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS = 0.312;
    static final double FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS = 0.35;
    static final double STRING_LENGTH_ADDITION_METERS = 0.143655638521;
    static final Rotation2d
            FIRST_JOINT_ANGLE_ADDITION = Rotation2d.fromDegrees(57.87 + 7.37 - 90),
            STRING_ANGLE_ADDITION = Rotation2d.fromDegrees(90 - 57.87);
    static final double DISTANCE_BETWEEN_JOINTS_METERS = 0.42;
    static final double CLOSED_STRING_LENGTH_METERS = 0.168;
    static final Rotation2d CLOSED_STRING_ANGLE = Rotation2d.fromDegrees(63.32);
    private static final Color8Bit
            BLUE = new Color8Bit(Color.kBlue),
            LIGHT_BLUE = new Color8Bit(Color.kLightBlue),
            GREEN = new Color8Bit(Color.kGreen),
            LIGHT_GREEN = new Color8Bit(Color.kLightGreen);
    static final Color8Bit GRAY = new Color8Bit(Color.kGray);
    static final double MECHANISM_LINE_WIDTH = 5;
    static final Rotation2d MECHANISM_STARTING_ANGLE = Rotation2d.fromDegrees(180);
    static final double STRING_CONNECTION_LIGAMENT_LENGTH = 0.07;
    static final Rotation2d STRING_CONNECTION_LIGAMENT_ANGLE = Rotation2d.fromDegrees(-50);
    static final Rotation2d SECOND_JOINT_ON_CHAIN_PITCH = Rotation2d.fromDegrees(90);
    static final double DRUM_DIAMETER_METERS = 0.04;
    static final ClimberVisualization
            RIGHT_MECHANISM = new ClimberVisualization(
            "RightClimberMechanism",
            BLUE,
            LIGHT_BLUE,
            RIGHT_CLIMBER_FIRST_JOINT_ORIGIN_POINT
    ),
            LEFT_MECHANISM = new ClimberVisualization(
                    "LeftClimberMechanism",
                    GREEN,
                    LIGHT_GREEN,
                    LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT
            );

    static final double CLIMBER_TOLERANCE_ROTATIONS = 0.01;

    static {
        configureMotor(RIGHT_MOTOR, RIGHT_MOTOR_INVERTED_VALUE, RIGHT_MOTOR_SIMULATION);
        configureMotor(LEFT_MOTOR, LEFT_MOTOR_INVERTED_VALUE, LEFT_MOTOR_SIMULATION);
    }

    private static void configureMotor(TalonFXMotor motor, InvertedValue invertedValue, SimpleMotorSimulation simulation) {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = GROUNDED_P;
        config.Slot0.kI = GROUNDED_I;
        config.Slot0.kD = GROUNDED_D;
        config.Slot0.kS = GROUNDED_KS;
        config.Slot0.kV = GROUNDED_KV;
        config.Slot0.kA = GROUNDED_KA;

        config.Slot1.kP = ON_CHAIN_P;
        config.Slot1.kI = ON_CHAIN_I;
        config.Slot1.kD = ON_CHAIN_D;
        config.Slot1.kS = ON_CHAIN_KS;
        config.Slot1.kV = ON_CHAIN_KV;
        config.Slot1.kA = ON_CHAIN_KA;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT_POSITION_ROTATIONS;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT_POSITION_ROTATIONS;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motor.applyConfiguration(config);
        motor.setPhysicsSimulation(simulation);

        motor.registerSignal(TalonFXSignal.POSITION, 100);
        motor.registerSignal(TalonFXSignal.VELOCITY, 100);
        motor.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        motor.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    public enum ClimberState {
        REST(0, false),
        PREPARE_FOR_CLIMB(0.5, false), //TODO: calibrate
        CLIMB(0.1, true); //TODO: calibrate

        public final double positionRotations;
        public final boolean affectedByRobotWeight;

        ClimberState(double positionRotations, boolean affectedByRobotWeight) {
            this.positionRotations = positionRotations;
            this.affectedByRobotWeight = affectedByRobotWeight;
        }
    }
}
