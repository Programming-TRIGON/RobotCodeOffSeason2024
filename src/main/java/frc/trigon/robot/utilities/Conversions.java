package frc.trigon.robot.utilities;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Conversions {
    public static final double
            MAG_TICKS = 4096,
            DEGREES_PER_ROTATIONS = 360,
            HUNDRED_MS_PER_SEC = 10,
            SEC_PER_MIN = 60;

    /**
     * Converts ticks from a Mag Encoder to rotations.
     *
     * @param magTicks ticks from a Mag Encoder
     * @return rotations
     */
    public static double magTicksToRotations(double magTicks) {
        return magTicks / MAG_TICKS;
    }

    /**
     * Converts rotations to Mag Encoder ticks.
     *
     * @param rotations rotations
     * @return Mag Encoder ticks
     */
    public static double rotationsToMagTicks(double rotations) {
        return rotations * MAG_TICKS;
    }

    /**
     * Converts degrees to rotations.
     *
     * @param degrees degrees
     * @return rotations
     */
    public static double degreesToRotations(double degrees) {
        return degrees / DEGREES_PER_ROTATIONS;
    }

    /**
     * Converts rotations to degrees.
     *
     * @param rotations rotations
     * @return degrees
     */
    public static double rotationsToDegrees(double rotations) {
        return rotations * DEGREES_PER_ROTATIONS;
    }

    /**
     * Converts ticks from a Mag Encoder to degrees.
     *
     * @param magTicks ticks from a Mag Encoder
     * @return degrees
     */
    public static double magTicksToDegrees(double magTicks) {
        return rotationsToDegrees(magTicksToRotations(magTicks));
    }

    /**
     * Converts degrees to Mag Encoder ticks.
     *
     * @param degrees degrees
     * @return Mag Encoder ticks
     */
    public static double degreesToMagTicks(double degrees) {
        return rotationsToMagTicks(degreesToRotations(degrees));
    }

    /**
     * Converts motor data to system data.
     * This can be velocity, position, acceleration, etc.
     *
     * @param motorData the motor data
     * @param gearRatio the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system
     *                  rotation.
     * @return the system data
     */
    public static double motorToSystem(double motorData, double gearRatio) {
        return motorData / gearRatio;
    }

    /**
     * Converts system data to motor data.
     * This can be velocity, position, acceleration, etc.
     *
     * @param systemData the system data
     * @param gearRatio  the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system
     *                   rotation.
     * @return the motor data
     */
    public static double systemToMotor(double systemData, double gearRatio) {
        return systemData * gearRatio;
    }

    /**
     * Applies an offset to a target position in order to compensate for a sensor offset.
     *
     * @param position the target position of the motor.
     * @param offset   the encoder value when the system is on zero position.
     * @return the offsetted position to give to the motor.
     */
    public static double offsetWrite(double position, double offset) {
        return position + offset;
    }

    /**
     * Applies an offset to the position read from the motor in order to compensate for a sensor offset.
     *
     * @param position the target position of the motor.
     * @param offset   the encoder value when the system is on zero position.
     * @return the actual position of the motor offset.
     */
    public static double offsetRead(double position, double offset) {
        return position - offset;
    }

    /**
     * Converts a frequency from per 100ms to per second.
     *
     * @param frequency the frequency per 100ms
     * @return the frequency per second
     */
    public static double perHundredMsToPerSecond(double frequency) {
        return frequency * HUNDRED_MS_PER_SEC;
    }

    /**
     * Converts a frequency from per second to per 100ms.
     *
     * @param frequency the frequency per second
     * @return the frequency per 100ms
     */
    public static double perSecondToPerHundredMs(double frequency) {
        return frequency / HUNDRED_MS_PER_SEC;
    }

    /**
     * Converts a frequency from per second to per minute.
     *
     * @param frequency the frequency per second
     * @return the frequency per minute
     */
    public static double perSecondToPerMinute(double frequency) {
        return frequency * SEC_PER_MIN;
    }

    /**
     * Converts a frequency from per minute to per second.
     *
     * @param frequency the frequency per minute
     * @return the frequency per second
     */
    public static double perMinuteToPerSecond(double frequency) {
        return frequency / SEC_PER_MIN;
    }

    /**
     * Converts a frequency from per 100ms to per minute.
     *
     * @param frequency the frequency per 100ms
     * @return the frequency per minute
     */
    public static double perHundredMsToPerMinute(double frequency) {
        return perSecondToPerMinute(perHundredMsToPerSecond(frequency));
    }

    /**
     * Converts a frequency from per minute to per 100ms.
     *
     * @param frequency the frequency per minute
     * @return the frequency per 100ms
     */
    public static double perMinToPerSec(double frequency) {
        return perSecondToPerHundredMs(frequency) / SEC_PER_MIN;
    }

    /**
     * Converts rotations to distance.
     *
     * @param rotations     the rotations
     * @param wheelDiameter the wheel diameter
     * @return the distance
     */
    public static double rotationsToDistance(double rotations, double wheelDiameter) {
        return rotations * wheelDiameter * Math.PI;
    }

    /**
     * Converts distance to rotations.
     *
     * @param distance      the distance
     * @param wheelDiameter the wheel diameter
     * @return the rotations
     */
    public static double distanceToRotations(double distance, double wheelDiameter) {
        return distance / (wheelDiameter * Math.PI);
    }

    /**
     * Converts a target output voltage to a percentage output when voltage compensation is enabled.
     * The voltage compensation saturation determines what voltage represents 100% output.
     * The compensated power is the voltage represented by a percentage of the saturation voltage.
     *
     * @param voltage    the target voltage output
     * @param saturation the configured saturation which represents 100% output
     * @return the percentage output to achieve the target voltage
     */
    public static double voltageToCompensatedPower(double voltage, double saturation) {
        return voltage / saturation;
    }

    /**
     * Converts a target output percentage output to voltage when voltage compensation is enabled.
     * The voltage compensation saturation determines what voltage represents 100% output.
     * The compensated power is the voltage represented by a percentage of the saturation voltage.
     *
     * @param power      the target percentage output
     * @param saturation the configured saturation which represents 100% output
     * @return the percentage output to achieve the target voltage
     */
    public static double compensatedPowerToVoltage(double power, double saturation) {
        return power * saturation;
    }

    /**
     * Scales a TrapezoidProfile.Constraints object by a given percentage.
     *
     * @param constraints the constraints to scale
     * @param percentage  the percentage of speed
     * @return the scaled constraints
     */
    public static TrapezoidProfile.Constraints scaleConstraints(TrapezoidProfile.Constraints constraints, double percentage) {
        return new TrapezoidProfile.Constraints(constraints.maxVelocity * (percentage / 100), constraints.maxAcceleration * (percentage / 100));
    }
}
