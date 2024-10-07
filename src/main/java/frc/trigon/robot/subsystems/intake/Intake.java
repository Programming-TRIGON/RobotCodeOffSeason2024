package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

import static frc.trigon.robot.subsystems.intake.IntakeConstants.DISTANCE_SENSOR;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor masterMotor = IntakeConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final StaticBrake staticBrakeRequest = new StaticBrake();
    private IntakeConstants.IntakeState targetState;

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
        DISTANCE_SENSOR.updateSensor();
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.MECHANISM.update(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
        Logger.recordOutput("HasNote", hasNote());
        Logger.recordOutput("distanceSensorScaledValue", DISTANCE_SENSOR.getScaledValue());
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
        targetState = IntakeConstants.IntakeState.STOP;
        IntakeConstants.MECHANISM.setTargetVelocity(0);
    }

    public IntakeConstants.IntakeState getTargetState() {
        return targetState;
    }

    public boolean hasNote() {
        return IntakeConstants.HAS_NOTE_BOOLEAN_EVENT.getAsBoolean();
    }

    /**
     * Checks if a note has been collected early using the motor's current.
     * This is quicker than {@linkplain Intake#hasNote} since it updates from the change in current (which happens right when we hit the note),
     * instead of the distance sensor which is positioned later on the system.
     *
     * @return whether an early note collection has been detected
     */
    public boolean isEarlyNoteCollectionDetected() {
        return IntakeConstants.EARLY_NOTE_COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
    }

    void sendStaticBrakeRequest() {
        masterMotor.setControl(staticBrakeRequest);
    }

    void setTargetState(IntakeConstants.IntakeState targetState) {
        this.targetState = targetState;
        if (targetState == IntakeConstants.IntakeState.FEED_SHOOTING || targetState == IntakeConstants.IntakeState.FEED_AMP || targetState == IntakeConstants.IntakeState.EJECT)
            getFeedingIndicationLEDsCommand().schedule();
        setTargetVoltage(targetState.voltage);
    }

    void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
        IntakeConstants.MECHANISM.setTargetVelocity(targetVoltage);
    }

    /**
     * Indicates to the driver that a note has been collected by rumbling the controller and flashing the robot's LEDs.
     */
    void indicateCollection() {
        if (!DriverStation.isAutonomous())
            OperatorConstants.DRIVER_CONTROLLER.rumble(IntakeConstants.RUMBLE_DURATION_SECONDS, IntakeConstants.RUMBLE_POWER);
        getCollectionIndicationLEDsCommand().schedule();
    }

    private Command getCollectionIndicationLEDsCommand() {
        return new SequentialCommandGroup(
                LEDStripCommands.getBlinkingCommand(Color.kOrange, IntakeConstants.COLLECTION_INDICATION_LEDS_BLINKING_INTERVAL_SECONDS, LEDStrip.LED_STRIPS).withTimeout(IntakeConstants.COLLECTION_INDICATION_BLINKING_TIME_SECONDS),
                LEDStripCommands.getStaticColorCommand(Color.kGreen, LEDStrip.LED_STRIPS)
        );
    }

    private Command getFeedingIndicationLEDsCommand() {
        return new SequentialCommandGroup(
                LEDStripCommands.getBlinkingCommand(Color.kYellow, IntakeConstants.FEEDING_INDICATION_LEDS_BLINKING_INTERVAL_SECONDS, LEDStrip.LED_STRIPS).withTimeout(IntakeConstants.FEEDING_INDICATION_BLINKING_TIME_SECONDS),
                LEDStripCommands.getStaticColorCommand(Color.kRed, LEDStrip.LED_STRIPS)
        );
    }
}