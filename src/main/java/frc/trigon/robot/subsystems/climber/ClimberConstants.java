package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.hardware.simulation.ElevatorSimulation;
import frc.trigon.robot.utilities.mechanisms.ElevatorMechanism2d;

public class ClimberConstants {
    private static final int
            MASTER_MOTOR_ID = 1,
            FOLLOWER_MOTOR_ID = 12;
    private static final String
            MASTER_MOTOR_NAME = "ClimberMasterMotor",
            FOLLOWER_MOTOR_NAME = "ClimberFollowerMotor";

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

    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = false;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    static final boolean ENABLE_FOC = true;
    static final double GEAR_RATIO = 1; //todo: ask mechanics for number
    private static final double
            P = 0,
            I = 0,
            D = 0,
            KS = 0,
            KG = 0,
            KV = 0,
            KA = 0;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MASS_KILOGRAMS = 1; //todo: get number from mechanics
    static final double
            DRUM_RADIUS_METERS = 0, //todo: get number from mechanics
            DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    static final double RETRACTED_CLIMBER_LENGTH_METERS = 0.1; //todo: get number from mechanics
    private static final double MAXIMUM_HEIGHT_METERS = 0.7; //todo: get number from mechanics
    private static final boolean SIMULATE_GRAVITY = true;
    private static final ElevatorSimulation SIMULATION = new ElevatorSimulation(
            GEARBOX,
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

    static final Pose3d CLIMBER_ORIGIN_POINT = new Pose3d(0.1, 0, 0.1, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-15), 0)); //todo: get numbers from mechanics
    static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d(
            "ClimberMechanism", MAXIMUM_HEIGHT_METERS, RETRACTED_CLIMBER_LENGTH_METERS, new Color8Bit(Color.kRed)
    );

    static {
        configureMasterMotor();
        configureFollowerMotor();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
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

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
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
}
