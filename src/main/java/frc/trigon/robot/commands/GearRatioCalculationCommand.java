package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

public class GearRatioCalculationCommand extends Command {
    private static final LoggedDashboardNumber MOVEMENT_VOLTAGE = new LoggedDashboardNumber("WheelRadiusCharacterization/Voltage", 1);
    private static final LoggedDashboardBoolean SHOULD_MOVE_CLOCKWISE = new LoggedDashboardBoolean("WheelRadiusCharacterization/ShouldMoveClockwise", false);

    private final Supplier rotorPositionSupplier;
    private final Supplier encoderPositionSupplier;
    private final DoubleConsumer runGearRatioCalculation;

    private double startingRotorPosition;
    private double startingEncoderPosition;
    private double gearRatio;

    public GearRatioCalculationCommand(Supplier rotorPositionSupplier, Supplier encoderPositionSupplier, DoubleConsumer runGearRatioCalculation, SubsystemBase requirement) {
        this.rotorPositionSupplier = rotorPositionSupplier;
        this.encoderPositionSupplier = encoderPositionSupplier;
        this.runGearRatioCalculation = runGearRatioCalculation;
        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        startingRotorPosition = (double) rotorPositionSupplier.get();
        startingEncoderPosition = (double) encoderPositionSupplier.get();
    }

    @Override
    public void execute() {
        runGearRatioCalculation.accept(MOVEMENT_VOLTAGE.get() * getRotationDirection());
        gearRatio = calculateGearRatio();

        Logger.recordOutput("GearRatioCalculation/RotorDistance", getRotorDistance());
        Logger.recordOutput("GearRatioCalculation/EncoderDistance", getEncoderDistance());
        Logger.recordOutput("GearRatioCalculation/GearRatio", gearRatio);
    }

    @Override
    public void end(boolean interrupted) {
        printResult();
    }

    private double calculateGearRatio() {
        final double currentRotorPosition = getRotorDistance();
        final double currentEncoderPosition = getEncoderDistance();
        return currentRotorPosition / currentEncoderPosition;
    }

    private double getRotorDistance() {
        return startingRotorPosition - (double) rotorPositionSupplier.get();
    }

    private double getEncoderDistance() {
        return startingEncoderPosition - (double) encoderPositionSupplier.get();
    }

    private int getRotationDirection() {
        return SHOULD_MOVE_CLOCKWISE.get() ? -1 : 1;
    }

    private void printResult() {
        System.out.println("Gear Ratio: " + gearRatio);
    }
}
