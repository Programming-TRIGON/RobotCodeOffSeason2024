package frc.trigon.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.simulation.MotorPhysicsSimulation;
import frc.trigon.robot.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.robot.utilities.Conversions;

public class SwerveModuleConstants {
    static final double WHEEL_DIAMETER_METERS = RobotConstants.IS_SIMULATION ? 0.1016 : 0.049149 * 2;
    static final double
            STEER_GEAR_RATIO = 12.8,
            DRIVE_GEAR_RATIO = 6.12;
    public static final double MAX_SPEED_ROTATIONS_PER_SECOND = Conversions.distanceToRotations(SwerveConstants.MAX_SPEED_METERS_PER_SECOND, WHEEL_DIAMETER_METERS);
    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = RobotConstants.IS_SIMULATION ? 0.1 : 0.1,
            DRIVE_CLOSED_LOOP_RAMP_RATE = RobotConstants.IS_SIMULATION ? 0.1 : 0.1;
    private static final InvertedValue
            DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final SensorDirectionValue STEER_ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue STEER_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final NeutralModeValue
            DRIVE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake,
            STEER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            DRIVE_SLIP_CURRENT = RobotConstants.IS_SIMULATION ? 100 : 100,
            STEER_CURRENT_LIMIT = RobotConstants.IS_SIMULATION ? 50 : 50;
    private static final double
            STEER_MOTOR_P = RobotConstants.IS_SIMULATION ? 75 : 75,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final double
            DRIVE_MOTOR_P = RobotConstants.IS_SIMULATION ? 50 : 50,
            DRIVE_MOTOR_I = 0,
            DRIVE_MOTOR_D = 0;

    private static final double
            DRIVE_MOMENT_OF_INERTIA = 0.003,
            STEER_MOMENT_OF_INERTIA = 0.003;
    private static final int
            DRIVE_MOTOR_AMOUNT = 1,
            STEER_MOTOR_AMOUNT = 1;
    private static final DCMotor
            DRIVE_MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(DRIVE_MOTOR_AMOUNT),
            STEER_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(STEER_MOTOR_AMOUNT);
    public static final MotorPhysicsSimulation
            DRIVE_SIMULATION = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            STEER_SIMULATION = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);

    public static final TalonFXConfiguration
            DRIVE_CONFIGURATION = generateDriveConfiguration(),
            STEER_CONFIGURATION = generateSteerConfiguration();
    public static final CANcoderConfiguration STEER_ENCODER_CONFIGURATION = generateSteerEncoderConfiguration();

    private static TalonFXConfiguration generateDriveConfiguration() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = DRIVE_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        config.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_SLIP_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_SLIP_CURRENT;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_SLIP_CURRENT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DRIVE_CLOSED_LOOP_RAMP_RATE;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = DRIVE_OPEN_LOOP_RAMP_RATE;

        config.Slot0.kP = DRIVE_MOTOR_P;
        config.Slot0.kI = DRIVE_MOTOR_I;
        config.Slot0.kD = DRIVE_MOTOR_D;

        return config;
    }

    private static TalonFXConfiguration generateSteerConfiguration() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = STEER_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = STEER_GEAR_RATIO;

        config.TorqueCurrent.PeakForwardTorqueCurrent = STEER_CURRENT_LIMIT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -STEER_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimit = STEER_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DRIVE_CLOSED_LOOP_RAMP_RATE;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = DRIVE_OPEN_LOOP_RAMP_RATE;

        config.Slot0.kP = STEER_MOTOR_P;
        config.Slot0.kI = STEER_MOTOR_I;
        config.Slot0.kD = STEER_MOTOR_D;

        return config;
    }

    private static CANcoderConfiguration generateSteerEncoderConfiguration() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = STEER_ENCODER_RANGE;
        config.MagnetSensor.SensorDirection = STEER_ENCODER_DIRECTION;

        return config;
    }
}
