package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.hardware.simulation.SimpleMotorSimulation;
import frc.trigon.robot.utilities.mechanisms.SpeedMechanism2d;

public class IntakeConstants {
    private static final int
            MASTER_MOTOR_ID = 1,
            FOLLOWER_MOTOR_ID = 2;
    private static final String
            MASTER_MOTOR_NAME = "MasterIntakeMotor",
            FOLLOWER_MOTOR_NAME = "FollowerIntakeMotor";
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
            MASTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive,
            FOLLOWER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final boolean FOLLOWER_MOTOR_OPPOSITE_DIRECTION = false;
    private static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final double GEAR_RATIO = 1; // get value from mechanics
    static final boolean FOC_ENABLED = true;

    private static final int MOTOR_AMOUNT = 2;
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 1; // get value from mechanics
    private static final SimpleMotorSimulation SIMULATION = new SimpleMotorSimulation(
            GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    private static final double MAX_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d MECHANISM = new SpeedMechanism2d(
            "IntakeMechanism", MAX_DISPLAYABLE_VELOCITY
    );

    static {
        ConfigureMasterMotor();
        configureFollowerMotor();
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

        MASTER_MOTOR.registerSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.MOTOR_VOLTAGE, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.TORQUE_CURRENT, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.POSITION, 100);
        MASTER_MOTOR.registerSignal(TalonFXSignal.VELOCITY, 100);
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
