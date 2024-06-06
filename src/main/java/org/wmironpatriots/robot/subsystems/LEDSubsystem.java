package org.wmironpatriots.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    AddressableLED LED = new AddressableLED(0);
    AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(120);

    Map<Integer, Command> ledEffectMap;

    public LEDSubsystem() {
        LED.setLength(LEDBuffer.getLength());

        // Set the data
        LED.setData(LEDBuffer);
        LED.start();
    }

    public void clearLEDs() {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, 0, 0, 0);
        }
        LED.setData(LEDBuffer);
    }

}