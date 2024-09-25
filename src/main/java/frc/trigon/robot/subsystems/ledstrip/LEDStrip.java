package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.commands.factories.GeneralCommands;

public class LEDStrip extends SubsystemBase {
    private static final AddressableLED LED = LEDStripConstants.LED;
    private final int numberOfLEDs;
    private final boolean inverted;
    private double rainbowFirstPixelHue = 0;
    private boolean areLEDsOnForBlinking = false;

    static {
        GeneralCommands.getDelayedCommand(
                1,
                () -> LEDStripConstants.LOW_BATTERY_TRIGGER.whileTrue(LEDStripCommands.getFlashCommand(Color.kRed))
        );
    }

    /**
     * Constructs a new LEDStrip.
     *
     * @param numberOfLEDs the number of LEDs in the strip
     * @param inverted     is the strip inverted
     */
    public LEDStrip(int numberOfLEDs, boolean inverted) {
        this.numberOfLEDs = numberOfLEDs;
        this.inverted = inverted;
    }

    public static void setDefaultCommandForAllLEDS(Command command) {
        LEDStripConstants.LED_STRIP.setDefaultCommand(command);
    }

    public int getNumberOfLEDS() {
        return numberOfLEDs;
    }

    private void setLedColors(edu.wpi.first.wpilibj.util.Color color, int index) {
        LEDStripConstants.LED_BUFFER.setLED(index, color);
        LED.setData(LEDStripConstants.LED_BUFFER);
    }

    private void setAllLedColors(Color color) {
        for (int index = 0; index < numberOfLEDs; index++) {
            setLedColors(color, index);
        }
    }

    void clearLedColors() {
        setAllLedColors(Color.kBlack);
    }

    void staticColor(Color color) {
        setAllLedColors(color);
    }


    void blink(Color color) {
        areLEDsOnForBlinking = !areLEDsOnForBlinking;
        if (areLEDsOnForBlinking)
            setAllLedColors(color);
        else
            clearLedColors();
    }

    void rainbow() {
        for (int led = 0; led < numberOfLEDs; led++) {
            final int hue = (int) (rainbowFirstPixelHue + (led * 180 / numberOfLEDs) % 180);
            LEDStripConstants.LED_BUFFER.setHSV(led, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    void threeSectionColor(Color firstSectionColor, Color secondSectionColor, Color thirdSectionColor) {
        final int firstLEDCount = (int) Math.floor(numberOfLEDs / 3.0);
        final int secondLEDCount = (int) Math.floor((numberOfLEDs - firstLEDCount) / 2.0);
        
        setThreeSectionColor(firstLEDCount, secondLEDCount, firstSectionColor, secondSectionColor, thirdSectionColor);
    }

    private void setThreeSectionColor(int firstLEDCount, int secondLEDCount, Color firstSectionColor, Color secondSectionColor, Color thirdSectionColor) {
        for (int i = 0; i < firstLEDCount; i++)
            setLedColors(inverted ? thirdSectionColor : firstSectionColor, i);
        for (int i = firstLEDCount; i < firstLEDCount + secondLEDCount; i++)
            setLedColors(secondSectionColor, i);
        for (int i = firstLEDCount + secondLEDCount; i < numberOfLEDs; i++)
            setLedColors(inverted ? firstSectionColor : thirdSectionColor, i);
    }
}