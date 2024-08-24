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
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public final class Autos {

    public static Command CJ(Drivetrain drivetrain, Shooter shooter, Intake intake) {
    // your welcome :)
    Command commandSequence = shooter.setShooterSpeedCommand(1)
      .andThen(new WaitCommand(2))
      .andThen(intake.setIntakeSpeedCommand(1)).andThen(new WaitCommand(1))
      .andThen(shooter.setShooterSpeedCommand(0))
      .andThen(intake.setIntakeSpeedCommand(0));
      //.andThen(drivetrain.getDriveCommand(-0.5, 0));


    return commandSequence;
  }

    public static Command Anna(Drivetrain drivetrain, Shooter shooter, Intake intake) {
    // your welcome :)
    Command commandSequence = shooter.setShooterSpeedCommand(1)
      .andThen(new WaitCommand(2))
      .andThen(intake.setIntakeSpeedCommand(1)).andThen(new WaitCommand(1))
      .andThen(shooter.setShooterSpeedCommand(0))
      .andThen(intake.setIntakeSpeedCommand(0));
      //.andThen(drivetrain.getDriveCommand(1, 0)).andThen(new WaitCommand(5)).andThen(drivetrain.getDriveCommand(0, 0));


    return commandSequence;
  }

    public static Command Max(Drivetrain drivetrain, Intake intake, Shooter shooter){
    //Command commandSequence = intake.setIntakeSpeedCommand(1);

      //Command commandSequence = drivetrain.feed();
    Command commandSequence = drivetrain.getDriveCommand(-0.5);
    // Command commandSequence = drivetrain.getDriveConcCommand(-0.5);
    // Command commandSequence = drivetrain.getDriveCommand(-0.5).andThen(new WaitCommand(3));
    // Command commandSequence = drivetrain.getDriveCommand(-0.5)

    //.andThen(drivetrain.getDriveCommand(-0.5)).withTimeout(3);
    // .andThen(new WaitCommand(2))
    // .andThen(intake.setIntakeSpeedCommand(1)).andThen(new WaitCommand(1))
    // .andThen(shooter.setShooterSpeedCommand(0))
    // .andThen(intake.setIntakeSpeedCommand(0));
    // //.andThen(drivetrain.getDriveContCommand(-0.5).withTimeout(3));
    //.andThen(drivetrain.feed());

    //.andThen(drivetrain.getDriveCommand(1));


    return commandSequence;
  }



  // public static Command TaxiAuto(Drivetrain drivetrain){
  //   return new RunCommand(() -> drivetrain.arcadeDrive(0, -0.5), drivetrain)
  //   .withTimeout(2);
  //  //.andThen(new RunCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain));
  // }

  //   //working
  // // public static Command Shoot(Drivetrain drivetrain, Shooter shooter, Intake intake){
  // //   return new RunCommand(() -> shooter.setShooterSpeed(1), shooter).withTimeout(2)
  // //   .andThen(new RunCommand(() -> intake.setIntakeSpeed(1), intake));

  // // }

  //   public static Command Shoot(Drivetrain drivetrain, Shooter shooter, Intake intake){
  //   return new RunCommand(() -> shooter.setShooterSpeed(0.5), shooter).andThen(new WaitCommand(1))
  //   .andThen(new RunCommand(() -> shooter.setShooterSpeed(0), shooter));
  //   // .andThen(new RunCommand(() -> intake.setIntakeSpeed(1), intake)).andThen(new WaitCommand(1))
  //   // .andThen(new RunCommand(() -> intake.setIntakeSpeed(0), intake));

  //   // .andThen(new RunCommand(() -> intake.setIntakeSpeed(0), intake))
  //   // .andThen(new RunCommand(() -> shooter.setShooterSpeed(0), shooter));
  //   //.finallyDo(new RunCommand(() -> shooter.setShooterSpeed(0), shooter));
  // }

  // public static Command Shoot_Stop_Test(Drivetrain drivetrain, Shooter shooter, Intake intake){
  //   return new RunCommand(()->shooter.setShooterSpeed(1), shooter).andThen(shooter.getStopShooterCommand());
  // }

  // public class a_Trial extends SequentialCommandGroup{
  //   public a_Trial(Intake intake, Shooter shooter, Drivetrain drivetrain){
  //     //addRequirements(intake, shooter, drivetrain);
  //     addCommands(
  //     new RunCommand(() ->shooter.getShootCommand()).withTimeout(2),
  //       //.andThen(intake.getReverseIntakeCommand()),
  //      new RunCommand(() -> intake.getIntakeCommand()).withTimeout(2),
  //      new RunCommand(() -> drivetrain.getDriveCommand(1)).withTimeout(2));

  //       //new RunCommand(() -> shooter.getShootCommand());
  //   }
  // }

  // public static Command a_bob(){
  //   return a_Trial;
  // }

  //public SequentialCommandGroup a_Sequ = new SequentialCommandGroup(new getIntakeCommand());

  //   public static Command a_Wait(Intake intake, Shooter shooter, Drivetrain drivetrain){
  //     return new RunCommand(() -> intake.getIntakeCommand().andThen(new WaitCommand(1).andThen(intake.getStopIntakeCommand())));
  //   }

  //   public static Command Test(Drivetrain drivetrain, Shooter shooter, Intake intake){
  //   return new RunCommand(() -> shooter.setShooterSpeed(0.5), shooter).withTimeout(2)
  //   .andThen(new RunCommand(() -> intake.setIntakeSpeed(0.5), intake).withTimeout(2))
  //   .andThen(new RunCommand(() -> shooter.setShooterSpeed(0), shooter)).withTimeout(2)
  //   .andThen(new RunCommand(() -> TaxiAuto(drivetrain)));
  //   // .andThen(new RunCommand(() -> drivetrain.arcadeDrive(0, 0.5), drivetrain))
  //   // .andThen(new RunCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain));
    

  // }


  // public static Command Two_Note(Drivetrain drivetrain, Shooter shooter, Intake intake){
  //   return new RunCommand(() -> shooter.setShooterSpeed(1), shooter);
  // }

  // public static Command Three_Note_Greedy(Drivetrain drivetrain, Shooter shooter, Intake intake){
  //   return new RunCommand(() -> shooter.setShooterSpeed(1), shooter);
  // }
}

//   public static Command ShootDistrupt(Drivetrain drivetrain, Shooter shooter, Intake intake){
//     return new RunCommand()
// }

//   public static Command FancyAuto(Drivetrain drivetrain){
//     return new RunCommand(() -> drivetrain.driveDistance(3, 1), drivetrain);
//   }


//   private Autos() {
//     throw new UnsupportedOperationException("This is a utility class!");
//   }


  
//}
