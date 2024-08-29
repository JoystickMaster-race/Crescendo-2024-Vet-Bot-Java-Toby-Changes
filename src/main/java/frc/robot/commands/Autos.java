// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public final class Autos {

  

    public static Command CJ(Drivetrain drivetrain, Shooter shooter, Intake intake) {
    // your welcome :)
    Command commandSequence = shooter.setShooterSpeedCommand(0)
      .andThen(intake.setIntakeSpeedCommand(0))
      .andThen(drivetrain.getDriveCommand(0))
      //.andThen(new WaitCommand(2))
      .andThen(shooter.setShooterSpeedCommand(1)).andThen(new WaitCommand(2))
      .andThen(intake.setIntakeSpeedCommand(0.8)).andThen(new WaitCommand(1))
      .andThen(shooter.setShooterSpeedCommand(0))
      .andThen(intake.setIntakeSpeedCommand(0))
      .andThen(drivetrain.getDriveCommand(-0.5)).andThen(new WaitCommand(2)).andThen(drivetrain.getDriveCommand(0))
      //.andThen(intake.setIntakeSpeedCommand(0))
      
      //.andThen(drivetrain.getDriveCommand(-0.5)).andThen(new WaitCommand(3)).andThen(drivetrain.getDriveCommand(0))
      // .andThen(drivetrain.getTurnCommand(-1)).andThen(new WaitCommand(0.69)).andThen(drivetrain.getTurnCommand(0))
      // .andThen(intake.setIntakeSpeedCommand(1))
      // .andThen(drivetrain.getDriveCommand(-0.5)).andThen(new WaitCommand(1.8)).andThen(drivetrain.getDriveCommand(0))
      // .andThen(intake.setIntakeSpeedCommand(0.2)).andThen(new WaitCommand(0.5)).andThen(intake.setIntakeSpeedCommand(-0.5)).andThen(new WaitCommand(0.5)).andThen(intake.setIntakeSpeedCommand(0))
      // .andThen(drivetrain.getDriveCommand(0.5)).andThen(new WaitCommand(2)).andThen(drivetrain.getDriveCommand(0))
      // .andThen(drivetrain.getTurnCommand(1)).andThen(new WaitCommand(1)).andThen(drivetrain.getTurnCommand(0))
      // .andThen(shooter.setShooterSpeedCommand(1)).andThen(new WaitCommand(3)).andThen(shooter.setShooterSpeedCommand(0))
      // .andThen(intake.setIntakeSpeedCommand(1)).andThen(new WaitCommand(2)).andThen(intake.setIntakeSpeedCommand(0))
      //.andThen(drivetrain.getDriveCommand(0.5)).andThen(new WaitCommand(2)).andThen(drivetrain.getDriveCommand(0))
  
      //.andThen(shooter.setShooterSpeedCommand(1)).andThen(new WaitCommand(2))

      // .andThen(intake.setIntakeSpeedCommand(0.85)).andThen(new WaitCommand(1)).andThen(intake.setIntakeSpeedCommand(0))
      // .andThen(shooter.setShooterSpeedCommand(0))
      // .andThen(intake.setIntakeSpeedCommand(0))
      // .andThen(drivetrain.getDriveCommand(0))

      .andThen(drivetrain.getFollow());
      // .andThen(drivetrain.backLeft.follow(drivetrain.frontLeft))
      // .andThen(drivetrain.backRight.follow(drivetrain.backLeft));
      // .andThen(drivetrain.getDriveCommand(-0.2)).andThen(new WaitCommand(1)).andThen(drivetrain.getDriveCommand(0))
      // .andThen(drivetrain.getTurnCommand(0.5)).andThen(new WaitCommand(1)).andThen(drivetrain.getTurnCommand(0));

      
      // .andThen(intake.setIntakeSpeedCommand(1)).andThen(new WaitCommand(2)).andThen(intake.setIntakeSpeedCommand(0))
      // .andThen(shooter.setShooterSpeedCommand(1)).andThen(drivetrain.getDriveCommand(0.5)).andThen(intake.setIntakeSpeedCommand(0))
      // .andThen(drivetrain.getDriveCommand(0));
      
      //.andThen(drivetrain.getDriveCommand(-0.5, 0));

    return commandSequence;
  }

    public static Command Anna(Drivetrain drivetrain, Shooter shooter, Intake intake) {
          Command commandSequence = (shooter.setShooterSpeedCommand(1)
      .andThen(new WaitCommand(2))
      .andThen(intake.setIntakeSpeedCommand(1)).andThen(new WaitCommand(1))
      .andThen(shooter.setShooterSpeedCommand(0))
      .andThen(intake.setIntakeSpeedCommand(0.0)).andThen(new WaitCommand(1))

      
      //.andThen(intake.setIntakeSpeedCommand(0))
      .andThen(drivetrain.getDriveCommand(-0.4)).andThen(new WaitCommand(2)).andThen(intake.setIntakeSpeedCommand(-0.8)).andThen(new WaitCommand(1)).andThen(drivetrain.getDriveCommand(0))

       .andThen(intake.setIntakeSpeedCommand(-0.8)).andThen(new WaitCommand(0.5)).andThen(intake.setIntakeSpeedCommand(0))
       //.andThen(intake.setIntakeSpeedCommand(0))
      // .andThen(drivetrain.getDriveCommand(0.5)).andThen(new WaitCommand(5)).andThen(drivetrain.getDriveCommand(0))
      // .andThen(shooter.setShooterSpeedCommand(1)).andThen(new WaitCommand(2))
      // .andThen(intake.setIntakeSpeedCommand(1)).andThen(new WaitCommand(2)).andThen(intake.setIntakeSpeedCommand(0))
      // .andThen(shooter.setShooterSpeedCommand(0))
      // .andThen(intake.setIntakeSpeedCommand(0))
      // .andThen(drivetrain.getDriveCommand(0))
      .andThen(drivetrain.getFollow()));

      // .andThen(drivetrain.getDriveCommand(-0.2)).andThen(new WaitCommand(1)).andThen(drivetrain.getDriveCommand(0))
      // .andThen(drivetrain.getTurnCommand(0.5)).andThen(new WaitCommand(0.6)).andThen(drivetrain.getTurnCommand(0))
      // .andThen(drivetrain.getDriveCommand(-0.5)).andThen(new WaitCommand(2)).andThen(drivetrain.getDriveCommand(0))
      // .andThen(intake.setIntakeSpeedCommand(0.5)).andThen(new WaitCommand(1)).andThen(intake.setIntakeSpeedCommand(0))

      // .andThen(drivetrain.getDriveCommand(0.5)).andThen(new WaitCommand(3)).andThen(drivetrain.getDriveCommand(0))
      // .andThen(shooter.setShooterSpeedCommand(1))
      // .andThen(intake.setIntakeSpeedCommand(1))).withTimeout(14).andThen(drivetrain.getDriveCommand(0));
    //.andThen(new WaitCommand(3)).andThen(intake.setIntakeSpeedCommand(0)).andThen(shooter.setShooterSpeedCommand(0)).andThen(drivetrain.getDriveCommand(0));
      
  // final DigitalInput m_beamSensor = new DigitalInput(7);

  // final BooleanSupplier beambroken = () -> !m_beamSensor.get();
  // final BooleanSupplier beamConnected = () -> m_beamSensor.get();

  // //   // your welcome :)
  //    Command commandSequence = intake.setIntakeSpeedCommand(1).until(beambroken).andThen(intake.setIntakeSpeedCommand(0.5)).until(beamConnected);
      

    
    //.andThen(drivetrain.getDriveCommand(1, 0)).andThen(new WaitCommand(5)).andThen(drivetrain.getDriveCommand(0, 0));

    return commandSequence;
  }

    public static Command Max(Drivetrain drivetrain, Intake intake, Shooter shooter){
    //Command commandSequence = intake.setIntakeSpeedCommand(1);

      Command commandSequence = drivetrain.getTurnCommand(0.5);
      //Command commandSequence = drivetrain.getDriveArcade(0.5);
    //Command commandSequence = drivetrain.getDriveContCommand(-0.5).withTimeout(500);

     //Command commandSequence = drivetrain.getDriveContCommand(-0.5);

  //Command commandSequence = drivetrain.getDriveCommand(-0.5).andThen(new WaitCommand(3));
    // Command commandSequence = drivetrain.getDriveCommand(-0.5)

    return commandSequence;
  }
}

//   private Autos() {
//     throw new UnsupportedOperationException("This is a utility class!");
//   }


  
//}
