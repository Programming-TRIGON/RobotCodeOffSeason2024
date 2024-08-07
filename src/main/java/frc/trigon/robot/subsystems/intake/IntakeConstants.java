package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.misc.objectdetectioncamera.SimulationObjectDetectionCameraIO;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class IntakeConstants {
    private static final int
            MASTER_MOTOR_ID = 1,
            FOLLOWER_MOTOR_ID = 2;
    private static final int DISTANCE_SENSOR_CHANNEL = 0;
    private static final String
            MASTER_MOTOR_NAME = "MasterIntakeMotor",
            FOLLOWER_MOTOR_NAME = "FollowerIntakeMotor",
            DISTANCE_SENSOR_NAME = "IntakeDistanceSensor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(
            MASTER_MOTOR_ID,
            MASTER_MOTOR_NAME,
            RobotConstants.CANIVORE_NAME
    ),
            FOLLOWER_MOTOR = new TalonFXMotor(
                    FOLLOWER_MOTOR_ID,
                    FOLLOWER_MOTOR_NAME,
                    RobotConstants.CANIVORE_NAME
            );
    static final SimpleSensor DISTANCE_SENSOR = SimpleSensor.createDutyCycleSensor(DISTANCE_SENSOR_CHANNEL, DISTANCE_SENSOR_NAME);

    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = false;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final double GEAR_RATIO = 0.003;
    private static final double
            DISTANCE_SENSOR_SCALING_SLOPE = 0.0004,
            DISTANCE_SENSOR_SCALING_INTERCEPT_POINT = 400;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final DoubleSupplier DISTANCE_SENSOR_SIMULATION_VALUE_SUPPLIER = SimulationObjectDetectionCameraIO.HAS_OBJECTS ? () -> 1 : () -> 0;

    private static final double MAX_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            "IntakeMechanism", MAX_DISPLAYABLE_VELOCITY
    );

    static final double NOTE_DISTANCE_THRESHOLD_METERS = 5;
    static final double RUMBLE_DURATION_SECONDS = 1;
    static final double RUMBLE_POWER = 1;

    static {
        ConfigureMasterMotor();
        configureFollowerMotor();
        configureDistanceSensor();
    }

    private static void ConfigureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSITE_DIRECTION);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    private static void configureDistanceSensor() {
        DISTANCE_SENSOR.setSimulationSupplier(DISTANCE_SENSOR_SIMULATION_VALUE_SUPPLIER);
        DISTANCE_SENSOR.setScalingConstants(DISTANCE_SENSOR_SCALING_SLOPE, DISTANCE_SENSOR_SCALING_INTERCEPT_POINT);
    }

    public enum IntakeState {
        COLLECTING(10),
        EJECTING(-4);

        public final double voltage;

        IntakeState(double voltage) {
            this.voltage = voltage;
        }
    }
}
