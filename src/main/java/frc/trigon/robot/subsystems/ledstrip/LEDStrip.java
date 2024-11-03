package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.factories.GeneralCommands;

import java.util.function.Supplier;

public class LEDStrip extends SubsystemBase {
    public static LEDStrip[] LED_STRIPS = new LEDStrip[0];
    private static final AddressableLED LED = LEDStripConstants.LED;
    private final int indexOffset;
    private final boolean inverted;
    private final int numberOfLEDs;
    private int lastBreatheLED;
    private double lastBreatheMovementTime = 0;
    private double rainbowFirstPixelHue = 0;
    private boolean areLEDsOnForBlinking = false;
    private double lastBlinkTime = 0;

    static {
        GeneralCommands.getDelayedCommand(
                1,
                () -> LEDStripConstants.LOW_BATTERY_TRIGGER.whileTrue(LEDStripCommands.getBlinkingCommand(Color.kRed, Color.kBlack, LEDStripConstants.LOW_BATTERY_BLINKING_INTERVAL_SECONDS, LED_STRIPS).withTimeout(LEDStripConstants.LOW_BATTERY_BLINKING_TIME_SECONDS))
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

        resetLEDSettings();
        addLEDStripToLEDStripsArray(this);
    }

    public static void setDefaultCommandForAllLEDS(Command command) {
        for (LEDStrip ledStrip : LED_STRIPS)
            ledStrip.setDefaultCommand(command);
    }

    public int getNumberOfLEDS() {
        return numberOfLEDs;
    }

    void resetLEDSettings() {
        lastBreatheLED = indexOffset;
        lastBreatheMovementTime = Timer.getFPGATimestamp();
        rainbowFirstPixelHue = 0;
        areLEDsOnForBlinking = false;
        lastBlinkTime = 0;
    }

    void clearLEDColors() {
        staticColor(Color.kBlack);
    }

    void blink(Color firstColor, Color secondColor, double blinkingIntervalSeconds) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastBlinkTime > blinkingIntervalSeconds) {
            lastBlinkTime = currentTime;
            areLEDsOnForBlinking = !areLEDsOnForBlinking;
        }
        if (areLEDsOnForBlinking)
            staticColor(firstColor);
        else
            staticColor(secondColor);
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

    void breathe(Color color, int breathingLEDs, double cycleTimeSeconds, boolean shouldLoop, boolean inverted) {
        clearLEDColors();
        double moveLEDTimeSeconds = cycleTimeSeconds / numberOfLEDs;
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastBreatheMovementTime > moveLEDTimeSeconds) {
            lastBreatheMovementTime = currentTime;
            if (inverted)
                lastBreatheLED--;
            else
                lastBreatheLED++;
        }
        if (inverted ? (lastBreatheLED < indexOffset) : (lastBreatheLED >= numberOfLEDs + indexOffset)) {
            if (!shouldLoop) {
                CommandConstants.DEFAULT_LEDS_COMMAND.schedule();
                return;
            }
            lastBreatheLED = inverted ? indexOffset + numberOfLEDs : indexOffset;
        }
        for (int i = 0; i < breathingLEDs; i++) {
            if (lastBreatheLED - i >= indexOffset && lastBreatheLED - i < indexOffset + numberOfLEDs)
                LEDStripConstants.LED_BUFFER.setLED(lastBreatheLED - i, color);
            else if (lastBreatheLED - i < indexOffset + numberOfLEDs)
                LEDStripConstants.LED_BUFFER.setLED(lastBreatheLED - i + numberOfLEDs, color);
        }
    }

    void sectionColor(int amountOfSections, Supplier<Color>[] colors) {
        if (amountOfSections != colors.length)
            throw new IllegalArgumentException("Amount of sections must be equal to the amount of colors");
        final int LEDSPerSection = (int) Math.floor(numberOfLEDs / amountOfSections);
        setSectionColor(amountOfSections, LEDSPerSection, colors);
    }

    private void setSectionColor(int amountOfSections, int LEDSPerSection, Supplier<Color>[] colors) {
        if (inverted) {
            for (int i = 0; i < amountOfSections; i++)
                setLEDColors(colors[amountOfSections - i - 1].get(), LEDSPerSection * i, i == amountOfSections - 1 ? numberOfLEDs - 1 : LEDSPerSection * (i + 1) - 1);
            return;
        }
        for (int i = 0; i < amountOfSections; i++)
            setLEDColors(colors[i].get(), LEDSPerSection * i, i == amountOfSections - 1 ? numberOfLEDs - 1 : LEDSPerSection * (i + 1) - 1);
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