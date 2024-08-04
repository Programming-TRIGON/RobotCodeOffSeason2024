package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.phoenix6.cancoder.CANcoderEncoder;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.hardware.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.utilities.mechanisms.SingleJointedArmMechanism2d;

public class PitcherConstants {
    private static final int
            MOTOR_ID = 0,
            ENCODER_ID = 0;
    private static final String
            MOTOR_NAME = "PitcherMotor",
            ENCODER_NAME = "PitcherEncoder";
    static final TalonFXMotor MOTOR = new TalonFXMotor(MOTOR_ID, MOTOR_NAME, RobotConstants.CANIVORE_NAME);
    static final CANcoderEncoder ENCODER = new CANcoderEncoder(ENCODER_ID, ENCODER_NAME, RobotConstants.CANIVORE_NAME);
    private static final InvertedValue INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            P = RobotConstants.IS_SIMULATION ? 1 : 1,
            I = RobotConstants.IS_SIMULATION ? 0 : 0,
            D = RobotConstants.IS_SIMULATION ? 0 : 0,
            KS = RobotConstants.IS_SIMULATION ? 0 : 0,
            KV = RobotConstants.IS_SIMULATION ? 0 : 0,
            KA = RobotConstants.IS_SIMULATION ? 0 : 0,
            KG = RobotConstants.IS_SIMULATION ? 0 : 0;
    private static final double
            MOTION_MAGIC_ACCELERATION = 0,
            MOTION_MAGIC_CRUISE_VELOCITY = 0;
    private static final FeedbackSensorSourceValue ENCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;
    private static final double GEAR_RATIO = 1;
    private static final SensorDirectionValue ENCODER_SENSOR_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;
    private static final double ENCODER_MAGNET_OFFSET_VALUE = 0;
    private static final AbsoluteSensorRangeValue ENCODER_ABSOLUTE_SENSOR_RANGE_VALUE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double
            PITCHER_LENGTH_METERS = 1,
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

    static final SingleJointedArmMechanism2d MECHANISM = new SingleJointedArmMechanism2d(
            "PitcherMechanism",
            PITCHER_LENGTH_METERS,
            new Color8Bit(Color.kBlue)
    );

    public static final Rotation2d DEFAULT_PITCH = Rotation2d.fromDegrees(0);

    static {
        configureMotor();
        configureEncoder();
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

        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
        config.Feedback.FeedbackSensorSource = ENCODER_TYPE;
        config.Feedback.RotorToSensorRatio = GEAR_RATIO;

        MOTOR.applyConfiguration(config);
        MOTOR.setPhysicsSimulation(SIMULATION);

        MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
    }

    public static void configureEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION_VALUE;
        config.MagnetSensor.MagnetOffset = ENCODER_MAGNET_OFFSET_VALUE;
        config.MagnetSensor.AbsoluteSensorRange = ENCODER_ABSOLUTE_SENSOR_RANGE_VALUE;

        ENCODER.applyConfiguration(config);
        ENCODER.setSimulationInputsFromTalonFX(MOTOR);
    }
}
