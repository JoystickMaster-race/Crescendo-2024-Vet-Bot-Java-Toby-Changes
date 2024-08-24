// Added as new file to robot/subsystems

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDDigitalOutputSubsystem extends SubsystemBase {
    private final DigitalOutput outputPin;

    public LEDDigitalOutputSubsystem(int channel) {
        outputPin = new DigitalOutput(channel);
    }

    public void setOutput(boolean state) {
        outputPin.set(state);
    }
}