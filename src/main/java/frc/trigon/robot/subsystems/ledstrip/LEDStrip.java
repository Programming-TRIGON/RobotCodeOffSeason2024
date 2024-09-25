package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.commands.factories.GeneralCommands;

public class LEDStrip extends SubsystemBase {
    private static final AddressableLED LED = LEDStripConstants.LED;
    private double rainbowFirstPixelHue = 0;
    private boolean areLEDsOnForBlinking = false;

    static {
        GeneralCommands.getDelayedCommand(
                1,
                () -> LEDStripConstants.LOW_BATTERY_TRIGGER.whileTrue(LEDStripCommands.getFlashCommand(Color.kRed))
        );
    }

    public LEDStrip() {
    }

    public static void setDefaultCommandForAllLEDS(Command command) {
        LEDStripConstants.LED_STRIP.setDefaultCommand(command);
    }

    public int getNumberOfLEDS() {
        return LEDStripConstants.NUMBER_OF_LEDS;
    }

    private void setLedColors(edu.wpi.first.wpilibj.util.Color color, int index) {
        LEDStripConstants.LED_BUFFER.setLED(index, color);
        LED.setData(LEDStripConstants.LED_BUFFER);
    }

    private void setAllLedColors(Color color) {
        for (int index = 0; index < LEDStripConstants.NUMBER_OF_LEDS; index++) {
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
        for (int led = 0; led < LEDStripConstants.NUMBER_OF_LEDS; led++) {
            final int hue = (int) (rainbowFirstPixelHue + (led * 180 / LEDStripConstants.NUMBER_OF_LEDS) % 180);
            LEDStripConstants.LED_BUFFER.setHSV(led, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    void threeSectionColor(Color firstSectionColor, Color secondSectionColor, Color thirdSectionColor) {
        final int ledsPerSection = (int) Math.floor(LEDStripConstants.NUMBER_OF_LEDS / 3.0);
        setThreeSectionColor(ledsPerSection, firstSectionColor, secondSectionColor, thirdSectionColor);
    }

    private void setThreeSectionColor(int ledsPerSection, Color firstSectionColor, Color secondSectionColor, Color thirdSectionColor) {
        for (int i = 0; i < ledsPerSection; i++)
            setLedColors(LEDStripConstants.INVERTED ? thirdSectionColor : firstSectionColor, i);
        for (int i = ledsPerSection; i < 2 * ledsPerSection * 2; i++)
            setLedColors(secondSectionColor, i);
        for (int i = 2 * ledsPerSection; i < LEDStripConstants.NUMBER_OF_LEDS; i++)
            setLedColors(LEDStripConstants.INVERTED ? firstSectionColor : thirdSectionColor, i);
    }
}