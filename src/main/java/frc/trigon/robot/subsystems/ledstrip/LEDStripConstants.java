package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.Robot;

public class LEDStripConstants {
    private static final int PORT = 9;
    private static final int
            RIGHT_CLIMBER_NUMBER_OF_LEDS = 22,
            LEFT_CLIMBER_NUMBER_OF_LEDS = 22;
    private static final boolean
            RIGHT_CLIMBER_INVERTED = false,
            LEFT_CLIMBER_INVERTED = false;
    static final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(RIGHT_CLIMBER_NUMBER_OF_LEDS + LEFT_CLIMBER_NUMBER_OF_LEDS);
    static final AddressableLED LED = new AddressableLED(PORT);

    static final double MINIMUM_BATTERY_VOLTAGE = 10.5;
    static final Trigger LOW_BATTERY_TRIGGER = new Trigger(() -> !DriverStation.isEnabled() && Robot.IS_REAL && RobotController.getBatteryVoltage() < LEDStripConstants.MINIMUM_BATTERY_VOLTAGE);
    static final Color
            LOW_BATTERY_FIRST_COLOR = Color.kOrange,
            LOW_BATTERY_SECOND_COLOR = Color.kYellow;
    static final double LOW_BATTERY_ALTERNATE_COLOR_INTERVAL_SECONDS = 0.2;
    static final double LOW_BATTERY_ALTERNATING_TIME_SECONDS = 10;

    public static final LEDStrip
            RIGHT_CLIMBER_LEDS = new LEDStrip(RIGHT_CLIMBER_INVERTED, RIGHT_CLIMBER_NUMBER_OF_LEDS, 0),
            LEFT_CLIMBER_LEDS = new LEDStrip(LEFT_CLIMBER_INVERTED, LEFT_CLIMBER_NUMBER_OF_LEDS, RIGHT_CLIMBER_NUMBER_OF_LEDS);

    static {
        LED.setLength(RIGHT_CLIMBER_NUMBER_OF_LEDS + LEFT_CLIMBER_NUMBER_OF_LEDS);
        LED.setData(LED_BUFFER);
        LED.start();
    }
}