package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.misc.objectdetectioncamera.SimulationObjectDetectionCameraIO;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.misc.simplesensor.SimpleSensor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SimpleMotorSimulation;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

import java.util.function.DoubleSupplier;

public class IntakeConstants {
    public static final int
            MASTER_MOTOR_ID = 16,
            FOLLOWER_MOTOR_ID = 17;
    private static final int DISTANCE_SENSOR_CHANNEL = 2;
    private static final String
            MASTER_MOTOR_NAME = "MasterIntakeMotor",
            FOLLOWER_MOTOR_NAME = "FollowerIntakeMotor",
            DISTANCE_SENSOR_NAME = "IntakeDistanceSensor";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME);
    static final SimpleSensor DISTANCE_SENSOR = SimpleSensor.createDutyCycleSensor(DISTANCE_SENSOR_CHANNEL, DISTANCE_SENSOR_NAME);

    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final boolean FOLLOWER_OPPOSES_MASTER = true;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final double GEAR_RATIO = 1.5;
    private static final double
            DISTANCE_SENSOR_SCALING_SLOPE = 0.0002,
            DISTANCE_SENSOR_SCALING_INTERCEPT_POINT = -200;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );
    private static final double NOTE_DISTANCE_THRESHOLD_CENTIMETERS = 14;
    private static final DoubleSupplier DISTANCE_SENSOR_SIMULATION_VALUE_SUPPLIER = () -> SimulationObjectDetectionCameraIO.HAS_OBJECTS ? NOTE_DISTANCE_THRESHOLD_CENTIMETERS - 1 : NOTE_DISTANCE_THRESHOLD_CENTIMETERS + 1;

    private static final double MAX_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            "IntakeMechanism", MAX_DISPLAYABLE_VELOCITY
    );

    public static final double RUMBLE_DURATION_SECONDS = 0.6;
    public static final double RUMBLE_POWER = 1;
    static final double NOTE_DETECTION_CONFIRMATION_DELAY_SECONDS = 0;
    static final BooleanEvent HAS_NOTE_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            () -> !RobotHardwareStats.isSimulation() && (DISTANCE_SENSOR.getScaledValue() < NOTE_DISTANCE_THRESHOLD_CENTIMETERS)
    ).debounce(NOTE_DETECTION_CONFIRMATION_DELAY_SECONDS);
    private static final double NOTE_COLLECTION_CURRENT = 50;
    private static final double NOTE_COLLECTION_TIME_THRESHOLD_SECONDS = 0.1;
    static final BooleanEvent EARLY_NOTE_COLLECTION_DETECTION_BOOLEAN_EVENT = new BooleanEvent(
            CommandScheduler.getInstance().getActiveButtonLoop(),
            () -> Math.abs(MASTER_MOTOR.getSignal(TalonFXSignal.TORQUE_CURRENT)) > IntakeConstants.NOTE_COLLECTION_CURRENT
    ).debounce(NOTE_COLLECTION_TIME_THRESHOLD_SECONDS);
    static final Color
            COLLECTION_INDICATION_BLINKING_FIRST_COLOR = Color.kOrangeRed,
            COLLECTION_INDICATION_BLINKING_SECOND_COLOR = Color.kBlack,
            FEEDING_INDICATION_COLOR = Color.kPurple,
            EJECTING_INDICATION_COLOR = Color.kDarkBlue;
    static final double COLLECTION_INDICATION_LEDS_BLINKING_INTERVAL_SECONDS = 0.2;
    static final double COLLECTION_INDICATION_BLINKING_TIME_SECONDS = 2;
    static final int
            FEEDING_INDICATION_BREATHING_LEDS_AMOUNT = 5,
            EJECTING_INDICATION_BREATHING_LEDS_AMOUNT = 9;
    static final double
            FEEDING_INDICATION_BREATHING_CYCLE_TIME_SECONDS = 0.05,
            EJECTING_INDICATION_BREATHING_CYCLE_TIME_SECONDS = 1;
    static final boolean
            FEEDING_INDICATION_BREATHING_SHOULD_LOOP = false,
            FEEDING_INDICATION_BREATHING_IS_INVERTED = false,
            EJECTING_INDICATION_BREATHING_SHOULD_LOOP = true,
            EJECTING_INDICATION_BREATHING_IS_INVERTED = true;
    public static final Color
            COLLECTION_BREATHING_LEDS_COLOR = Color.kOrangeRed,
            NOTE_DETECTION_CAMERA_HAS_TARGETS_BREATHING_LEDS_COLOR = Color.kGreen,
            NOTE_DETECTION_CAMERA_HAS_NO_TARGETS_BREATHING_LEDS_COLOR = Color.kRed;
    public static final int
            COLLECTION_BREATHING_LEDS_AMOUNT = 5,
            ALIGN_TO_NOTE_BREATHING_LEDS_AMOUNT = 5;
    public static final double
            COLLECTION_BREATHING_CYCLE_TIME_SECONDS = 0.2,
            ALIGN_TO_NOTE_BREATHING_CYCLE_TIME_SECONDS = 0.2;
    public static final boolean
            COLLECTION_BREATHING_SHOULD_LOOP = true,
            COLLECTION_BREATHING_IS_INVERTED = false,
            ALIGN_TO_NOTE_BREATHING_SHOULD_LOOP = true,
            ALIGN_TO_NOTE_BREATHING_IS_INVERTED = false;

    static {
        configureMasterMotor();
        configureFollowerMotor();
        configureDistanceSensor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitEnable = false;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.REVERSE_LIMIT, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitEnable = false;

        FOLLOWER_MOTOR.applyConfiguration(config);

        FOLLOWER_MOTOR.registerSignal(TalonFXSignal.REVERSE_LIMIT, 100);

        final Follower followerRequest = new Follower(MASTER_MOTOR_ID, FOLLOWER_OPPOSES_MASTER);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    private static void configureDistanceSensor() {
        DISTANCE_SENSOR.setSimulationSupplier(DISTANCE_SENSOR_SIMULATION_VALUE_SUPPLIER);
        DISTANCE_SENSOR.setScalingConstants(DISTANCE_SENSOR_SCALING_SLOPE, DISTANCE_SENSOR_SCALING_INTERCEPT_POINT);
    }

    public enum IntakeState {
        COLLECT(4.5),
        EJECT(-2),
        STOP(0),
        FEED_SHOOTING(7),
        FEED_AMP(5),
        CORRECT_NOTE_POSITION(-1.1);

        public final double voltage;

        IntakeState(double voltage) {
            this.voltage = voltage;
        }
    }
}