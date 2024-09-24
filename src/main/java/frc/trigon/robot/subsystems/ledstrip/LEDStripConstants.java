package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.Robot;

public class LEDStripConstants {
    private static final int PORT = 0;
    private static final int NUMBER_OF_LEDS = 24;
    private static final boolean INVERTED = false;
    static final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(NUMBER_OF_LEDS);
    static final AddressableLED LED = new AddressableLED(PORT);

    static final double MINIMUM_BATTERY_VOLTAGE = 10.5;
    static final Trigger LOW_BATTERY_TRIGGER = new Trigger(() -> !DriverStation.isEnabled() && Robot.IS_REAL && RobotController.getBatteryVoltage() < LEDStripConstants.MINIMUM_BATTERY_VOLTAGE);

    static final double FLASHING_TIME_SECONDS = 1;
    static final double BLINKING_INTERVAL_SECONDS = 0.2;

    public static final LEDStrip LED_STRIP = new LEDStrip(NUMBER_OF_LEDS, INVERTED);

    static {
        LED.setLength(NUMBER_OF_LEDS);
        LED.setData(LED_BUFFER);
        LED.start();
    }
}