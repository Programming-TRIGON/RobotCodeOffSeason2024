package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.cancoder.CANcoderEncoder;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.SingleJointedArmSimulation;
import org.trigon.utilities.mechanisms.DoubleJointedArmMechanism2d;

public class PitcherConstants {
    private static final int
            MASTER_MOTOR_ID = 11,
            FOLLOWER_MOTOR_ID = 12,
            ENCODER_ID = 0;
    private static final String
            MASTER_MOTOR_NAME = "MasterPitcherMotor",
            FOLLOWER_MOTOR_NAME = "FollowerPitcherMotor",
            ENCODER_NAME = "PitcherEncoder";
    static final TalonFXMotor
            MASTER_MOTOR = new TalonFXMotor(MASTER_MOTOR_ID, MASTER_MOTOR_NAME, RobotConstants.CANIVORE_NAME),
            FOLLOWER_MOTOR = new TalonFXMotor(FOLLOWER_MOTOR_ID, FOLLOWER_MOTOR_NAME, RobotConstants.CANIVORE_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME, RobotConstants.CANIVORE_NAME);

    private static final InvertedValue
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = false;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            P = RobotHardwareStats.isSimulation() ? 10 : 1,
            I = RobotHardwareStats.isSimulation() ? 0 : 0,
            D = RobotHardwareStats.isSimulation() ? 0 : 0,
            KS = RobotHardwareStats.isSimulation() ? 0 : 0,
            KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            KA = RobotHardwareStats.isSimulation() ? 0 : 0,
            KG = RobotHardwareStats.isSimulation() ? 0 : 0;
    private static final GravityTypeValue GRAVITY_TYPE_VALUE = GravityTypeValue.Arm_Cosine;
    private static final StaticFeedforwardSignValue STATIC_FEEDFORWARD_SIGN_VALUE = StaticFeedforwardSignValue.UseVelocitySign;
    private static final double
            MOTION_MAGIC_ACCELERATION = 50,
            MOTION_MAGIC_CRUISE_VELOCITY = 50;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double GEAR_RATIO = 1;
    private static final SensorDirectionValue ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;
    private static final double ENCODER_MAGNET_OFFSET_VALUE = 0;
    private static final AbsoluteSensorRangeValue ENCODER_ABSOLUTE_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double
            PITCHER_LENGTH_METERS = 0.5,
            PITCHER_MASS_KILOGRAMS = 0.5;
    private static final Rotation2d
            PITCHER_MINIMUM_ANGLE = Rotation2d.fromDegrees(0),
            PITCHER_MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);
    private static final SingleJointedArmSimulation SIMULATION = new SingleJointedArmSimulation(
            GEARBOX,
            GEAR_RATIO,
            PITCHER_LENGTH_METERS,
            PITCHER_MASS_KILOGRAMS,
            PITCHER_MINIMUM_ANGLE,
            PITCHER_MAXIMUM_ANGLE,
            true
    );

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Second.of(1)),
            Units.Volts.of(2),
            Units.Second.of(1000)
    );

    public static final DoubleJointedArmMechanism2d PITCHER_AND_AMP_ALIGNER_MECHANISM = new DoubleJointedArmMechanism2d(
            "PitcherAndAmpAlignerMechanism",
            PITCHER_LENGTH_METERS,
            AmpAlignerConstants.AMP_ALIGNER_LENGTH_METERS,
            new Color8Bit(Color.kGreen)
    );

    public static final Rotation2d DEFAULT_PITCH = Rotation2d.fromDegrees(0);
    static final Rotation2d PITCH_TOLERANCE = Rotation2d.fromDegrees(0.6);

    static {
        configureMasterMotor();
        configureFollowerMotor();
        configureEncoder();
    }

    private static void configureMasterMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = MASTER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;
        config.Slot0.kG = KG;
        config.Slot0.GravityType = GRAVITY_TYPE_VALUE;
        config.Slot0.StaticFeedforwardSign = STATIC_FEEDFORWARD_SIGN_VALUE;

        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;
        config.Feedback.RotorToSensorRatio = GEAR_RATIO;

        MASTER_MOTOR.applyConfiguration(config);
        MASTER_MOTOR.setPhysicsSimulation(SIMULATION);

        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.STATOR_CURRENT, 100);
    }

    private static void configureFollowerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = FOLLOWER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;

        FOLLOWER_MOTOR.applyConfiguration(config);

        final Follower followerRequest = new Follower(MASTER_MOTOR_ID, FOLLOWER_MOTOR_OPPOSITE_DIRECTION);
        FOLLOWER_MOTOR.setControl(followerRequest);
    }

    public static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ENCODER_MAGNET_OFFSET_VALUE;
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_ABSOLUTE_SENSOR_RANGE_VALUE;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MASTER_MOTOR);
    }
}
