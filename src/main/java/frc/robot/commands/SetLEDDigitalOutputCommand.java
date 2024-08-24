// Added as new file to robot/commands
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDDigitalOutputSubsystem;

public class SetLEDDigitalOutputCommand extends CommandBase {
    private final LEDDigitalOutputSubsystem LEDOutputSubsystem;
    private final boolean state;

    public SetLEDDigitalOutputCommand(LEDDigitalOutputSubsystem LEDOutputSubsystem, boolean state) {
        this.LEDOutputSubsystem = LEDOutputSubsystem;
        this.state = state;
        addRequirements(LEDOutputSubsystem);
    }

    @Override
    public void initialize() {
        LEDOutputSubsystem.setOutput(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}