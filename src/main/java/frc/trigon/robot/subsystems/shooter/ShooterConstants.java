package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.hardware.simulation.FlywheelSimulation;
import org.trigon.utilities.mechanisms.SpeedMechanism2d;

public class ShooterConstants {
    private static final int
            RIGHT_MOTOR_ID = 9,
            LEFT_MOTOR_ID = 10;
    private static final String
            RIGHT_MOTOR_NAME = "RightShootingMotor",
            LEFT_MOTOR_NAME = "LeftShootingMotor";
    static final TalonFXMotor
            RIGHT_MOTOR = new TalonFXMotor(RIGHT_MOTOR_ID, RIGHT_MOTOR_NAME),
            LEFT_MOTOR = new TalonFXMotor(LEFT_MOTOR_ID, LEFT_MOTOR_NAME);

    private static final InvertedValue
            RIGHT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            LEFT_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final double
            P = RobotHardwareStats.isSimulation() ? 15 : 0,
            I = RobotHardwareStats.isSimulation() ? 0 : 0,
            D = RobotHardwareStats.isSimulation() ? 0 : 0,
            KS = RobotHardwareStats.isSimulation() ? 0.35586 : 0,
            KV = RobotHardwareStats.isSimulation() ? 0 : 0,
            KA = RobotHardwareStats.isSimulation() ? 0.59136 : 0;
    private static final double GEAR_RATIO = 1;

    private static final int
            RIGHT_MOTOR_AMOUNT = 1,
            LEFT_MOTOR_AMOUNT = 1;
    private static final DCMotor
            RIGHT_GEARBOX = DCMotor.getKrakenX60Foc(RIGHT_MOTOR_AMOUNT),
            LEFT_GEARBOX = DCMotor.getKrakenX60Foc(LEFT_MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    static final FlywheelSimulation
            RIGHT_SIMULATION = new FlywheelSimulation(RIGHT_GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA),
            LEFT_SIMULATION = new FlywheelSimulation(LEFT_GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA);

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Second),
            Units.Volts.of(7),
            Units.Second.of(1000)
    );

    private static final double MAX_DISPLAYABLE_VELOCITY = 100;
    static final SpeedMechanism2d
            RIGHT_MECHANISM = new SpeedMechanism2d("RightShooterMechanism", MAX_DISPLAYABLE_VELOCITY),
            LEFT_MECHANISM = new SpeedMechanism2d("LeftShooterMechanism", MAX_DISPLAYABLE_VELOCITY);

    public static final double WHEEL_DIAMETER_METERS = edu.wpi.first.math.util.Units.inchesToMeters(4);
    public static final double AMP_SHOOTING_VELOCTY_ROTATIONS_PER_SECOND = 5;//TODO: Calibrate on real robot
    static final double RIGHT_MOTOR_TO_LEFT_MOTOR_RATIO = 1.3;
    static final double VELOCITY_TOLERANCE = 0.3;

    static {
        configureMotor(RIGHT_MOTOR, RIGHT_MOTOR_INVERTED_VALUE, RIGHT_SIMULATION);
        configureMotor(LEFT_MOTOR, LEFT_MOTOR_INVERTED_VALUE, LEFT_SIMULATION);
    }

    private static void configureMotor(TalonFXMotor motor, InvertedValue invertedValue, FlywheelSimulation simulation) {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = false;

        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NEUTRAL_MODE_VALUE;

        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;
        config.Slot0.kA = KA;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        motor.applyConfiguration(config);
        motor.setPhysicsSimulation(simulation);

        motor.registerSignal(TalonFXSignal.VELOCITY, 100);
        motor.registerSignal(TalonFXSignal.POSITION, 100);
        motor.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        motor.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
    }
}
