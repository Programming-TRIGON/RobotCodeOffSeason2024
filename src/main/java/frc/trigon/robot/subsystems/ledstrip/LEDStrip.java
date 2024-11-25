package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.commands.factories.GeneralCommands;

import java.util.function.Supplier;

public class LEDStrip extends SubsystemBase {
    public static LEDStrip[] LED_STRIPS = new LEDStrip[0];
    private static final AddressableLED LED = LEDStripConstants.LED;
    private final int indexOffset;
    private final boolean inverted;
    private final int numberOfLEDs;
    private int lastBreatheLED;
    private double lastLEDMovementTime = 0;
    private double rainbowFirstPixelHue = 0;
    private boolean areLEDsOnForBlinking = false;
    private boolean alternateColor = true;
    private int amountOfColorFlowLEDs = 0;

    static {
        GeneralCommands.getDelayedCommand(
                1,
                () -> LEDStripConstants.LOW_BATTERY_TRIGGER.whileTrue(LEDStripCommands.getAlternateColorCommand(
                        LEDStripConstants.LOW_BATTERY_FIRST_COLOR,
                        LEDStripConstants.LOW_BATTERY_SECOND_COLOR,
                        LEDStripConstants.LOW_BATTERY_ALTERNATE_COLOR_INTERVAL_SECONDS,
                        LED_STRIPS
                ).withTimeout(LEDStripConstants.LOW_BATTERY_ALTERNATING_TIME_SECONDS))
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
        lastLEDMovementTime = Timer.getFPGATimestamp();
        rainbowFirstPixelHue = 0;
        areLEDsOnForBlinking = false;
        alternateColor = true;
        amountOfColorFlowLEDs = 0;
    }

    void clearLEDColors() {
        staticColor(Color.kBlack);
    }

    void blink(Color firstColor, Color secondColor, double blinkingIntervalSeconds) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastLEDMovementTime > blinkingIntervalSeconds) {
            lastLEDMovementTime = currentTime;
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
        if (inverted) {
            rainbowFirstPixelHue -= 3;
            if (rainbowFirstPixelHue < 0)
                rainbowFirstPixelHue += 180;
            return;
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    void breathe(Color color, int breathingLEDs, double cycleTimeSeconds, boolean shouldLoop, boolean inverted) {
        clearLEDColors();
        inverted = this.inverted != inverted;
        double moveLEDTimeSeconds = cycleTimeSeconds / numberOfLEDs;
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastLEDMovementTime > moveLEDTimeSeconds) {
            lastLEDMovementTime = currentTime;
            if (inverted)
                lastBreatheLED--;
            else
                lastBreatheLED++;
        }
        if (inverted ? (lastBreatheLED < indexOffset) : (lastBreatheLED >= numberOfLEDs + indexOffset)) {
            if (!shouldLoop) {
                getDefaultCommand().schedule();
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

    void colorFlow(Color color, double cycleTimeSeconds, boolean shouldLoop, boolean inverted) {
        clearLEDColors();
        inverted = this.inverted != inverted;
        double moveLEDTimeSeconds = cycleTimeSeconds / numberOfLEDs;
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastLEDMovementTime > moveLEDTimeSeconds) {
            lastLEDMovementTime = currentTime;
            if (inverted)
                amountOfColorFlowLEDs--;
            else
                amountOfColorFlowLEDs++;
        }
        if (inverted ? amountOfColorFlowLEDs < 0 : amountOfColorFlowLEDs >= numberOfLEDs) {
            if (!shouldLoop) {
                getDefaultCommand().schedule();
                return;
            }
            amountOfColorFlowLEDs = inverted ? numberOfLEDs : 0;
        }
        if (inverted) {
            setLEDColors(color, numberOfLEDs - amountOfColorFlowLEDs, numberOfLEDs - 1);
            return;
        }
        setLEDColors(color, 0, amountOfColorFlowLEDs);
    }

    void alternateColor(Color firstColor, Color secondColor, double intervalSeconds) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastLEDMovementTime > intervalSeconds) {
            alternateColor = !alternateColor;
            lastLEDMovementTime = currentTime;
        }
        if (alternateColor) {
            for (int i = 0; i < numberOfLEDs; i++)
                LEDStripConstants.LED_BUFFER.setLED(i + indexOffset, i % 2 == 0 ? firstColor : secondColor);
            return;
        }
        for (int i = 0; i < numberOfLEDs; i++)
            LEDStripConstants.LED_BUFFER.setLED(i + indexOffset, i % 2 == 0 ? secondColor : firstColor);
    }

    void sectionColor(int amountOfSections, Supplier<Color>[] colors) {
        if (amountOfSections != colors.length)
            throw new IllegalArgumentException("Amount of sections must be equal to the amount of colors");
        final int LEDSPerSection = (int) Math.floor(numberOfLEDs / amountOfSections);
        setSectionColor(amountOfSections, LEDSPerSection, colors);

        for (int i = 0; i < amountOfSections; i++)
            setLEDColors(colors[i].get(), LEDSPerSection * i, i == amountOfSections - 1 ? numberOfLEDs - 1 : LEDSPerSection * (i + 1) - 1);
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