package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase{
    WPI_VictorSPX m_intakeMotor;
    WPI_VictorSPX m_indexMotor;
    DigitalInput m_beamBreak;

    public Intake(){
        m_intakeMotor = new WPI_VictorSPX(kIntakeID);
        m_indexMotor = new WPI_VictorSPX(kIndexerID);
        m_beamBreak = new DigitalInput(kBeamBreakID);

        m_indexMotor.follow(m_intakeMotor);
    }

    public Command getIntakeCommand() {
        return this.startEnd(
            ()-> {
                setIntakeSpeed(kIntakeSpeed);
            },
            
            ()-> {
                stop();
            });
}

    public Command getReverseIntakeCommand() {
        return this.startEnd(
            ()-> {
                setIntakeSpeed(kReverseIntakeSpeed);
            },
            
            ()-> {
                stop();
            });
}

public void setIntakeSpeed(double speed){
    m_intakeMotor.set(speed);
}

// public void setIntakeVoltage(){
//     m_intakeMotor.setVoltage(kIntakeVoltage);
// }
public boolean getBeamBreak(){
    return m_beamBreak.get();
}
public void stop(){
    m_intakeMotor.set(0);
}
}


