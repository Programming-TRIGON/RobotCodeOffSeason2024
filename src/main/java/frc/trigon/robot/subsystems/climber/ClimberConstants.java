package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.utilities.mechanisms.ElevatorMechanism2d;

public class ClimberConstants {
    private static final int
            RIGHT_MOTOR_ID = 14,
            LEFT_MOTOR_ID = 15;
    private static final String
            RIGHT_MOTOR_NAME = "RightClimberMotor",
            LEFT_MOTOR_NAME = "LeftClimberMotor";
    static final TalonFXMotor
            RIGHT_MOTOR = new TalonFXMotor(
            RIGHT_MOTOR_ID,
            RIGHT_MOTOR_NAME,
            RobotConstants.CANIVORE_NAME
    ),
            LEFT_MOTOR = new TalonFXMotor(
                    LEFT_MOTOR_ID,
                    LEFT_MOTOR_NAME,
                    RobotConstants.CANIVORE_NAME
            );

    private static final InvertedValue
            RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            LEFT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    static final boolean ENABLE_FOC = true;
    static final double //TODO: calibrate
            MAX_NON_CLIMBING_VELOCITY = 1,
            MAX_NON_CLIMBING_ACCELERATION = 1,
            MAX_CLIMBING_VELOCITY = 1,
            MAX_CLIMBING_ACCELERATION = 1;
    private static final double //TODO: calibrate
            GROUNDED_P = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            GROUNDED_KA = RobotHardwareStats.isSimulation() ? 0 : 0;
    static final double GROUNDED_KG = RobotHardwareStats.isSimulation() ? 0 : 1;
    private static final double //TODO: calibrate
            ON_CHAIN_P = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_I = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_D = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            ON_CHAIN_KA = RobotHardwareStats.isSimulation() ? 0 : 0;
    static final double ON_CHAIN_KG = RobotHardwareStats.isSimulation() ? 0 : 1;
    static final int
            GROUNDED_SLOT = 0,
            ON_CHAIN_SLOT = 1;
    static final double GEAR_RATIO = 1; //TODO: ask mechanics for number

    private static final int
            RIGHT_MOTOR_AMOUNT = 1,
            LEFT_MOTOR_AMOUNT = 1;
    private static final DCMotor
            RIGHT_GEARBOX = DCMotor.getFalcon500Foc(RIGHT_MOTOR_AMOUNT),
            LEFT_GEARBOX = DCMotor.getFalcon500Foc(LEFT_MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation
            RIGHT_MOTOR_SIMULATION = new SimpleMotorSimulation(
            RIGHT_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    ),
            LEFT_MOTOR_SIMULATION = new SimpleMotorSimulation(
                    LEFT_GEARBOX,
                    GEAR_RATIO,
                    MOMENT_OF_INERTIA
            );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Second.of(1)),
            Units.Volts.of(8),
            null,
            null
    );

    static final Translation3d
            RIGHT_CLIMBER_FIRST_JOINT_ORIGIN_POINT = new Translation3d(0.1, 0, 0.1), //TODO: get numbers from mechanics
            LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT = new Translation3d(0.1, 0, 0.1); //TODO: get numbers from mechanics
    static final double FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS = 0.312;
    static final double FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS = 0.343;
    static final double STRING_LENGTH_ADDITION = 0.143655638521;
    static final double ANGLE_ADDITION = 57.87 + 7.37 - 90;
    static final double RETRACTED_CLIMBER_LENGTH_METERS = 0.1; //TODO: get number from mechanics
    static final double MAXIMUM_HEIGHT_METERS = 0.7; //TODO: get number from mechanics
    static final ElevatorMechanism2d
            RIGHT_MECHANISM = new ElevatorMechanism2d(
            "RightClimberMechanism", MAXIMUM_HEIGHT_METERS, RETRACTED_CLIMBER_LENGTH_METERS, new Color8Bit(Color.kRed)
    ),
            LEFT_MECHANISM = new ElevatorMechanism2d(
                    "LeftClimberMechanism", MAXIMUM_HEIGHT_METERS, RETRACTED_CLIMBER_LENGTH_METERS, new Color8Bit(Color.kRed)
            );

    static final double DRUM_DIAMETER_METERS = 0.04; //TODO: get number from mechanics

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
        config.Slot0.kG = GROUNDED_KG;
        config.Slot0.kA = GROUNDED_KA;

        config.Slot1.kP = ON_CHAIN_P;
        config.Slot1.kI = ON_CHAIN_I;
        config.Slot1.kD = ON_CHAIN_D;
        config.Slot1.kS = ON_CHAIN_KS;
        config.Slot1.kV = ON_CHAIN_KV;
        config.Slot1.kG = ON_CHAIN_KG;
        config.Slot1.kA = ON_CHAIN_KA;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motor.applyConfiguration(config);
        motor.setPhysicsSimulation(simulation);

        motor.registerSignal(TalonFXSignal.POSITION, 100);
        motor.registerSignal(TalonFXSignal.VELOCITY, 100);
        motor.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        motor.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    public enum ClimberState {
        RESTING(0, false),
        PREPARE_FOR_CLIMBING(0.2, false), //TODO: calibrate
        CLIMBING(0.7, true); //TODO: calibrate

        public final double positionMeters;
        public final boolean affectedByRobotWeight;

        ClimberState(double positionMeters, boolean affectedByRobotWeight) {
            this.positionMeters = positionMeters;
            this.affectedByRobotWeight = affectedByRobotWeight;
        }
    }
}