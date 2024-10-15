package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.commands.factories.GeneralCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;

public class LEDStrip extends SubsystemBase {
    public static LEDStrip[] LED_STRIPS = new LEDStrip[0];
    private static final AddressableLED LED = LEDStripConstants.LED;
    private final int indexOffset;
    private final boolean inverted;
    private final int numberOfLEDs;
    private int lastBreatheLED;
    private double rainbowFirstPixelHue = 0;
    private boolean areLEDsOnForBlinking = false;
    private double lastBreatheMovementTime = 0;
    private double lastBlinkTime = 0;

    static {
        GeneralCommands.getDelayedCommand(
                1,
                () -> LEDStripConstants.LOW_BATTERY_TRIGGER.whileTrue(LEDStripCommands.getBlinkingCommand(Color.kRed, LEDStripConstants.LOW_BATTERY_BLINKING_INTERVAL_SECONDS, LED_STRIPS).withTimeout(LEDStripConstants.LOW_BATTERY_BLINKING_TIME_SECONDS))
        );
    }

    @Override
    public void periodic() {
        LED.setData(LEDStripConstants.LED_BUFFER);
    }

    public LEDStrip(boolean inverted, int numberOfLEDs, int indexOffset) {
        this.indexOffset = indexOffset;
        this.inverted = inverted;
        this.numberOfLEDs = numberOfLEDs;

        lastBreatheLED = indexOffset;
        addLEDStripToLEDStripsArray(this);
    }

    public static void setDefaultCommandForAllLEDS(Command command) {
        for (LEDStrip ledStrip : LED_STRIPS)
            ledStrip.setDefaultCommand(command);
    }

    public static void changeDefaultCommandForAllLEDs(Command newDefaultCommand) {
        for (LEDStrip ledStrip : LED_STRIPS) {
            final Command currentDefaultCommand = ledStrip.getDefaultCommand();
            if (currentDefaultCommand != null)
                currentDefaultCommand.cancel();
            ledStrip.setDefaultCommand(newDefaultCommand);
        }
    }

    public int getNumberOfLEDS() {
        return numberOfLEDs;
    }

    void clearLedColors() {
        staticColor(Color.kBlack);
    }

    void blink(Color color, double blinkingIntervalSeconds) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastBlinkTime > blinkingIntervalSeconds) {
            lastBlinkTime = currentTime;
            areLEDsOnForBlinking = !areLEDsOnForBlinking;
        }
        if (areLEDsOnForBlinking)
            staticColor(color);
        else
            clearLedColors();
    }

    void staticColor(Color color) {
        setLEDColors(color, 0, numberOfLEDs - 1);
    }

    void rainbow() {
        for (int led = 0; led < numberOfLEDs; led++) {
            final int hue = (int) (rainbowFirstPixelHue + (led * 180 / numberOfLEDs) % 180);
            LEDStripConstants.LED_BUFFER.setHSV(led + indexOffset, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    void breathe(Color color, int breathingLEDs) {
        clearLedColors();
        double moveLEDTimeSeconds = IntakeConstants.FEEDING_INDICATION_BREATHING_TIME_SECONDS / numberOfLEDs;
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastBreatheMovementTime > moveLEDTimeSeconds) {
            lastBreatheMovementTime = currentTime;
            lastBreatheLED++;
        }
        if (lastBreatheLED >= numberOfLEDs + indexOffset)
            lastBreatheLED = indexOffset;
        for (int i = 0; i < breathingLEDs; i++) {
            if (lastBreatheLED - i >= indexOffset)
                LEDStripConstants.LED_BUFFER.setLED(lastBreatheLED - i, color);
            else
                LEDStripConstants.LED_BUFFER.setLED(lastBreatheLED - i + numberOfLEDs, color);
        }
    }

    void threeSectionColor(Color firstSectionColor, Color secondSectionColor, Color thirdSectionColor) {
        final int ledsPerSection = (int) Math.floor(numberOfLEDs / 3.0);
        setThreeSectionColor(ledsPerSection, firstSectionColor, secondSectionColor, thirdSectionColor);
    }

    private void setThreeSectionColor(int ledsPerSection, Color firstSectionColor, Color secondSectionColor, Color thirdSectionColor) {
        setLEDColors(inverted ? thirdSectionColor : firstSectionColor, 0, ledsPerSection);
        setLEDColors(secondSectionColor, ledsPerSection, ledsPerSection * 2);
        setLEDColors(inverted ? firstSectionColor : thirdSectionColor, ledsPerSection * 2, numberOfLEDs - 1);
    }

    private void setLEDColors(Color color, int startIndex, int endIndex) {
        for (int i = 0; i <= endIndex - startIndex; i++)
            LEDStripConstants.LED_BUFFER.setLED(startIndex + indexOffset + i, color);
    }

    private void addLEDStripToLEDStripsArray(LEDStrip ledStrip) {
        final LEDStrip[] newLEDStrips = new LEDStrip[LED_STRIPS.length + 1];
        System.arraycopy(LED_STRIPS, 0, newLEDStrips, 0, LED_STRIPS.length);
        newLEDStrips[LED_STRIPS.length] = ledStrip;
        LED_STRIPS = newLEDStrips;
    }
}