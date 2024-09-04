package frc.trigon.robot.subsystems;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.factories.GeneralCommands;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.trigon.hardware.RobotHardwareStats;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.function.Consumer;

/**
 * A class that represents a subsystem that has motors (rather than something like LEDs).
 * This class will automatically stop all the motors when the robot is disabled, and set the motors to brake when the robot is enabled.
 * If a subsystem doesn't need to ever brake (i.e. shooter, flywheel, etc.), then it should override the {@link #setBrake(boolean)} method and do nothing.
 */
public abstract class MotorSubsystem extends edu.wpi.first.wpilibj2.command.SubsystemBase {
    private static final List<MotorSubsystem> REGISTERED_SUBSYSTEMS = new ArrayList<>();
    private static final Trigger DISABLED_TRIGGER = new Trigger(DriverStation::isDisabled);
    private static final LoggedDashboardBoolean ENABLE_EXTENSIVE_LOGGING = new LoggedDashboardBoolean("EnableExtensiveLogging", true);

    static {
        DISABLED_TRIGGER.onTrue(new InstantCommand(() -> forEach(MotorSubsystem::stop)).ignoringDisable(true));
        DISABLED_TRIGGER.onFalse(new InstantCommand(() -> {
            setAllSubsystemsBrakeAsync(true);
            CommandConstants.STATIC_WHITE_LED_COLOR_COMMAND.cancel();
            GeneralCommands.IS_BRAKING = true;
        }).ignoringDisable(true));
    }

    private final SysIdRoutine sysIdRoutine = createSysIdRoutine();

    public MotorSubsystem() {
        REGISTERED_SUBSYSTEMS.add(this);
    }

    /**
     * Runs periodically, to update the subsystem, and update the mechanism of the subsystem (if there is one).
     * This only updates the mechanism if the robot is in replay mode or extensive logging is enabled.
     * This function cannot be overridden. Use {@linkplain MotorSubsystem#updatePeriodically} or {@linkplain MotorSubsystem#updateMechanism} (depending on the usage) instead.
     */
    @Override
    public final void periodic() {
        updatePeriodically();
        if (RobotHardwareStats.isReplay())
            updateMechanism();
        else if (ENABLE_EXTENSIVE_LOGGING.get())
            updateMechanism();
    }

    /**
     * Runs the given consumer on all the subsystem instances.
     *
     * @param toRun the consumer to run on each registered subsystem
     */
    public static void forEach(Consumer<MotorSubsystem> toRun) {
        REGISTERED_SUBSYSTEMS.forEach(toRun);
    }

    /**
     * Sets whether the all the subsystems should brake or coast their motors.
     * This command will run asynchronously, since the TalonFX's setting of brake/coast is blocking.
     *
     * @param brake whether the motors should brake or coast
     */
    public static void setAllSubsystemsBrakeAsync(boolean brake) {
        CompletableFuture.runAsync(() -> forEach((subsystem) -> subsystem.setBrake(brake)));
    }

    public static boolean isExtensiveLoggingEnabled() {
        return ENABLE_EXTENSIVE_LOGGING.get();
    }

    /**
     * Creates a quasistatic (ramp up) command for characterizing the subsystem's mechanism.
     *
     * @param direction the direction in which to run the test
     * @return the command
     * @throws IllegalStateException if the {@link MotorSubsystem#getSysIdConfig()} function wasn't overridden or returns null
     */
    public final Command getQuasistaticCharacterizationCommand(SysIdRoutine.Direction direction) throws IllegalStateException {
        if (sysIdRoutine == null)
            throw new IllegalStateException("Subsystem " + getName() + " doesn't have a SysId routine!");
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Creates a dynamic (constant "step up") command for characterizing the subsystem's mechanism.
     *
     * @param direction the direction in which to run the test
     * @return the command
     * @throws IllegalStateException if the {@link MotorSubsystem#getSysIdConfig()} function wasn't overridden or returns null
     */
    public final Command getDynamicCharacterizationCommand(SysIdRoutine.Direction direction) throws IllegalStateException {
        if (sysIdRoutine == null)
            throw new IllegalStateException("Subsystem " + getName() + " doesn't have a SysId routine!");
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Sets whether the subsystem's motors should brake or coast.
     * If a subsystem doesn't need to ever brake (i.e. shooter, flywheel, etc.), don't implement this method.
     *
     * @param brake whether the motors should brake or coast
     */
    public void setBrake(boolean brake) {
    }

    /**
     * Drives the motor with the given voltage for characterizing.
     *
     * @param voltageMeasure the target voltage
     */
    public void drive(Measure<Voltage> voltageMeasure) {
    }

    /**
     * Updates the SysId log of the motor states for characterizing.
     *
     * @param log the log to update
     */
    public void updateLog(SysIdRoutineLog log) {
    }

    /**
     * Updates the mechanism of the subsystem periodically if the robot is in replay mode, or if {@linkplain MotorSubsystem#ENABLE_EXTENSIVE_LOGGING) is true.
     * This wouldn't always run, in order to save resources.
     */
    public void updateMechanism() {
    }

    /**
     * Updates periodically. Anything that should be updated periodically but isn't related to the mechanism (or shouldn't always be logged, in order to save resources) should be put here.
     */
    public void updatePeriodically() {
    }

    public SysIdRoutine.Config getSysIdConfig() {
        return null;
    }

    public void changeDefaultCommand(Command newDefaultCommand) {
        final Command currentDefaultCommand = getDefaultCommand();
        if (currentDefaultCommand != null)
            currentDefaultCommand.cancel();
        setDefaultCommand(newDefaultCommand);
    }

    public abstract void stop();

    private SysIdRoutine createSysIdRoutine() {
        if (getSysIdConfig() == null)
            return null;

        return new SysIdRoutine(
                getSysIdConfig(),
                new SysIdRoutine.Mechanism(
                        this::drive,
                        this::updateLog,
                        this,
                        getName()
                )
        );
    }
}