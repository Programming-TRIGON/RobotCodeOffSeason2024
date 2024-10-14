package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;

public class ClimberConstants {
    public static final int
            RIGHT_MOTOR_ID = 14,
            LEFT_MOTOR_ID = 15;
    private static final String
            RIGHT_MOTOR_NAME = "RightClimberMotor",
            LEFT_MOTOR_NAME = "LeftClimberMotor";
    static final TalonFXMotor
            RIGHT_MOTOR = new TalonFXMotor(RIGHT_MOTOR_ID, RIGHT_MOTOR_NAME),
            LEFT_MOTOR = new TalonFXMotor(LEFT_MOTOR_ID, LEFT_MOTOR_NAME);

    private static final InvertedValue
            RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            LEFT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    static final boolean ENABLE_FOC = true;
    private static final double //TODO: calibrate
            LEFT_GROUNDED_P = RobotHardwareStats.isSimulation() ? 800 : 1,
            LEFT_GROUNDED_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            LEFT_GROUNDED_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            LEFT_GROUNDED_KS = RobotHardwareStats.isSimulation() ? 0.0045028 : 0.078964,
            LEFT_GROUNDED_KV = RobotHardwareStats.isSimulation() ? 8.792 : 7.9056,
            LEFT_GROUNDED_KA = RobotHardwareStats.isSimulation() ? 0.17809 : 0.18439;
    private static final double //TODO: calibrate
            RIGHT_GROUNDED_P = RobotHardwareStats.isSimulation() ? 800 : 4.5626,
            RIGHT_GROUNDED_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            RIGHT_GROUNDED_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            RIGHT_GROUNDED_KS = RobotHardwareStats.isSimulation() ? 0.0045028 : 0.079947,
            RIGHT_GROUNDED_KV = RobotHardwareStats.isSimulation() ? 8.792 : 7.9986,
            RIGHT_GROUNDED_KA = RobotHardwareStats.isSimulation() ? 0.17809 : 0.21705;
    private static final double //TODO: calibrate
            ON_CHAIN_P = RobotHardwareStats.isSimulation() ? RIGHT_GROUNDED_P : 4.5626,
            ON_CHAIN_I = RobotHardwareStats.isSimulation() ? LEFT_GROUNDED_I : 0,
            ON_CHAIN_D = RobotHardwareStats.isSimulation() ? LEFT_GROUNDED_D : 0,
            ON_CHAIN_KS = RobotHardwareStats.isSimulation() ? LEFT_GROUNDED_KS : 0.079947,
            ON_CHAIN_KV = RobotHardwareStats.isSimulation() ? LEFT_GROUNDED_KV : 7.9986,
            ON_CHAIN_KA = RobotHardwareStats.isSimulation() ? LEFT_GROUNDED_KA : 0.21705;
    static final double
            MAX_GROUNDED_VELOCITY = RobotHardwareStats.isSimulation() ? 12 / LEFT_GROUNDED_KV : 1,
            MAX_GROUNDED_ACCELERATION = RobotHardwareStats.isSimulation() ? 12 / LEFT_GROUNDED_KA : 1,
            MAX_ON_CHAIN_VELOCITY = RobotHardwareStats.isSimulation() ? (12 / ON_CHAIN_KV) - 0.75 : 1,
            MAX_ON_CHAIN_ACCELERATION = RobotHardwareStats.isSimulation() ? (12 / ON_CHAIN_KA) - 50 : 1;
    static final int
            GROUNDED_SLOT = 0,
            ON_CHAIN_SLOT = 1;
    private static final double FORWARD_SOFT_LIMIT_POSITION_ROTATIONS = -2.9;
    private static final double LIMIT_SWITCH_RESET_POSITION = 0;
    static final double GEAR_RATIO = 68.57;

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
            Units.Volts.of(5),
            null,
            null
    );

    static final Translation3d
            RIGHT_CLIMBER_FIRST_JOINT_ORIGIN_POINT = new Translation3d(0.295, -0.2545, 0.27445),
            LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT = new Translation3d(0.295, 0.2545, 0.27445);
    static final double FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS = 0.312;
    static final double FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS = 0.35;
    static final double STRING_LENGTH_ADDITION_METERS = 0.143655638521;
    static final Rotation2d
            FIRST_JOINT_ANGLE_ADDITION = Rotation2d.fromDegrees(57.87 + 7.37 - 90),
            STRING_ANGLE_ADDITION = Rotation2d.fromDegrees(90 - 57.87);
    static final double DISTANCE_BETWEEN_JOINTS_METERS = 0.42;
    static final Rotation2d CLOSED_STRING_ANGLE = Rotation2d.fromDegrees(63.32);
    private static final Color8Bit
            BLUE = new Color8Bit(Color.kBlue),
            LIGHT_BLUE = new Color8Bit(Color.kLightBlue),
            GREEN = new Color8Bit(Color.kGreen),
            LIGHT_GREEN = new Color8Bit(Color.kLightGreen);
    static final Color8Bit GRAY = new Color8Bit(Color.kGray);
    static final double MECHANISM_LINE_WIDTH = 5;
    static final Rotation2d MECHANISM_STARTING_ANGLE = Rotation2d.fromDegrees(180);
    static final Rotation2d SECOND_JOINT_ON_CHAIN_PITCH = Rotation2d.fromDegrees(90);
    static final double DRUM_DIAMETER_METERS = 0.04;
    static final ClimberVisualization
            RIGHT_VISUALIZATION = new ClimberVisualization(
            "RightClimber",
            BLUE,
            LIGHT_BLUE,
            RIGHT_CLIMBER_FIRST_JOINT_ORIGIN_POINT
    ),
            LEFT_VISUALIZATION = new ClimberVisualization(
                    "LeftClimber",
                    GREEN,
                    LIGHT_GREEN,
                    LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT
            );

    public static final double
            MOVE_CLIMBER_DOWN_VOLTAGE = -4,
            MOVE_CLIMBER_UP_VOLTAGE = 4;
    static final double CLIMBER_TOLERANCE_ROTATIONS = 0.01;

    static {
        configureMotor(RIGHT_MOTOR, RIGHT_MOTOR_INVERTED_VALUE, RIGHT_MOTOR_SIMULATION, RIGHT_GROUNDED_P, RIGHT_GROUNDED_I, RIGHT_GROUNDED_D, RIGHT_GROUNDED_KS, RIGHT_GROUNDED_KV, RIGHT_GROUNDED_KA, IntakeConstants.MASTER_MOTOR_ID);
        configureMotor(LEFT_MOTOR, LEFT_MOTOR_INVERTED_VALUE, LEFT_MOTOR_SIMULATION, LEFT_GROUNDED_P, LEFT_GROUNDED_I, LEFT_GROUNDED_D, LEFT_GROUNDED_KS, LEFT_GROUNDED_KV, LEFT_GROUNDED_KA, IntakeConstants.FOLLOWER_MOTOR_ID);
    }

    private static void configureMotor(TalonFXMotor motor, InvertedValue invertedValue, SimpleMotorSimulation simulation, double groundedP, double groundedI, double groundedD, double groundedKS, double groundedKV, double groundedKA, int limitSwitchID) {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = groundedP;
        config.Slot0.kI = groundedI;
        config.Slot0.kD = groundedD;
        config.Slot0.kS = groundedKS;
        config.Slot0.kV = groundedKV;
        config.Slot0.kA = groundedKA;

        config.Slot1.kP = ON_CHAIN_P;
        config.Slot1.kI = ON_CHAIN_I;
        config.Slot1.kD = ON_CHAIN_D;
        config.Slot1.kS = ON_CHAIN_KS;
        config.Slot1.kV = ON_CHAIN_KV;
        config.Slot1.kA = ON_CHAIN_KA;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = FORWARD_SOFT_LIMIT_POSITION_ROTATIONS;
        config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = limitSwitchID;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = LIMIT_SWITCH_RESET_POSITION;

        config.MotionMagic.MotionMagicAcceleration = MAX_GROUNDED_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_GROUNDED_VELOCITY;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motor.applyConfiguration(config);
        motor.setPhysicsSimulation(simulation);

        motor.registerSignal(TalonFXSignal.POSITION, 100);
        motor.registerSignal(TalonFXSignal.VELOCITY, 100);
        motor.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        motor.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        motor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        motor.registerSignal(TalonFXSignal.REVERSE_LIMIT, 100);
        motor.registerSignal(TalonFXSignal.FORWARD_LIMIT, 100);
    }

    public enum ClimberState {
        REST(0, false),
        PREPARE_FOR_CLIMB(-2.9, false),
        CLIMB(-0.1, true);

        public final double positionRotations;
        public final boolean affectedByRobotWeight;

        ClimberState(double positionRotations, boolean affectedByRobotWeight) {
            this.positionRotations = positionRotations;
            this.affectedByRobotWeight = affectedByRobotWeight;
        }
    }
}