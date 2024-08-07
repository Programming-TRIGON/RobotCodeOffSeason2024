package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.ElevatorSimulation;
import org.trigon.utilities.mechanisms.ElevatorMechanism2d;

public class ClimberConstants {
    private static final int
            RIGHT_MOTOR_ID = 1,
            LEFT_MOTOR_ID = 12;
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
    static final double GEAR_RATIO = 1; //TODO: ask mechanics for number
    private static final double
            P = RobotHardwareStats.isSimulation() ? 0 : 0,
            I = RobotHardwareStats.isSimulation() ? 0 : 0,
            D = RobotHardwareStats.isSimulation() ? 0 : 0,
            KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            KG = RobotHardwareStats.isSimulation() ? 0 : 0,
            KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            KA = RobotHardwareStats.isSimulation() ? 0 : 0;

    private static final int
            RIGHT_MOTOR_AMOUNT = 1,
            LEFT_MOTOR_AMOUNT = 1;
    private static final DCMotor
            RIGHT_GEARBOX = DCMotor.getFalcon500Foc(RIGHT_MOTOR_AMOUNT),
            LEFT_GEARBOX = DCMotor.getFalcon500Foc(LEFT_MOTOR_AMOUNT);
    private static final double MASS_KILOGRAMS = 1; //TODO: get number from mechanics
    static final double
            DRUM_RADIUS_METERS = 0, //TODO: get number from mechanics
            DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    static final double RETRACTED_CLIMBER_LENGTH_METERS = 0.1; //TODO: get number from mechanics
    private static final double MAXIMUM_HEIGHT_METERS = 0.7; //TODO: get number from mechanics
    private static final boolean SIMULATE_GRAVITY = true;
    private static final ElevatorSimulation
            RIGHT_MOTOR_SIMULATION = new ElevatorSimulation(
            RIGHT_GEARBOX,
            GEAR_RATIO,
            MASS_KILOGRAMS,
            DRUM_RADIUS_METERS,
            RETRACTED_CLIMBER_LENGTH_METERS,
            MAXIMUM_HEIGHT_METERS,
            SIMULATE_GRAVITY
    ),
            LEFT_MOTOR_SIMULATION = new ElevatorSimulation(
                    LEFT_GEARBOX,
                    GEAR_RATIO,
                    MASS_KILOGRAMS,
                    DRUM_RADIUS_METERS,
                    RETRACTED_CLIMBER_LENGTH_METERS,
                    MAXIMUM_HEIGHT_METERS,
                    SIMULATE_GRAVITY
            );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Second.of(1)),
            Units.Volts.of(8),
            null,
            null
    );

    static final Pose3d
            RIGHT_CLIMBER_ORIGIN_POINT = new Pose3d(0.1, 0, 0.1, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-15), 0)), //TODO: get numbers from mechanics
            LEFT_CLIMBER_ORIGIN_POINT = new Pose3d(0.1, 0, 0.1, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-15), 0)); //TODO: get numbers from mechanics
    static final ElevatorMechanism2d
            RIGHT_MECHANISM = new ElevatorMechanism2d(
            "RightClimberMechanism", MAXIMUM_HEIGHT_METERS, RETRACTED_CLIMBER_LENGTH_METERS, new Color8Bit(Color.kRed)
    ),
            LEFT_MECHANISM = new ElevatorMechanism2d(
                    "LeftClimberMechanism", MAXIMUM_HEIGHT_METERS, RETRACTED_CLIMBER_LENGTH_METERS, new Color8Bit(Color.kRed)
            );

    static {
        configureMotor(RIGHT_MOTOR, RIGHT_MOTOR_INVERTED_VALUE, RIGHT_MOTOR_SIMULATION);
        configureMotor(LEFT_MOTOR, LEFT_MOTOR_INVERTED_VALUE, LEFT_MOTOR_SIMULATION);
    }

    private static void configureMotor(TalonFXMotor motor, InvertedValue invertedValue, ElevatorSimulation simulation) {

        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kG = KG;
        config.Slot0.kA = KA;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motor.applyConfiguration(config);
        motor.setPhysicsSimulation(simulation);

        motor.registerSignal(TalonFXSignal.POSITION, 100);
        motor.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        motor.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    public enum ClimberState {
        RETRACTED(0),
        CLIMBING(0.7);

        public final double positionMeters;

        ClimberState(double positionMeters) {
            this.positionMeters = positionMeters;
        }
    }
}
