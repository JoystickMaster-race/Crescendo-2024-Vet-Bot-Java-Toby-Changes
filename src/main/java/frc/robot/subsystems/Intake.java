// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import static frc.robot.Constants.IntakeConstants.*;

// public class Intake extends SubsystemBase{
//     WPI_VictorSPX m_intakeMotor;
//     WPI_VictorSPX m_indexMotor;

//     public Intake(){
//         m_intakeMotor = new WPI_VictorSPX(kIntakeID);
//         m_indexMotor = new WPI_VictorSPX(kIndexerID);

//         m_indexMotor.follow(m_intakeMotor);
//     }

//     public Command getIntakeCommand() {
//         return this.startEnd(
//             ()-> {
//                 setIntakeSpeed(kIntakeSpeed);
//             },
            
//             ()-> {
//                 stop();
//             });
// }

// public void setIntakeSpeed(double speed){
//     m_intakeMotor.set(speed);
// }

// public void stop(){
//     m_intakeMotor.set(0);
// }
// }


