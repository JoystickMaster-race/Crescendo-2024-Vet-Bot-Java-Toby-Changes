// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import java.lang.Math;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

/* 
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class Drivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */

  //Drivebase moving components
  DifferentialDrive m_drivetrain;
  public WPI_TalonSRX frontLeft;
  public WPI_TalonSRX frontRight;
  public WPI_TalonSRX backLeft;
  public WPI_TalonSRX backRight;

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public Drivetrain() {
    frontLeft = new WPI_TalonSRX(2);
    frontRight = new WPI_TalonSRX(3);
    backLeft = new WPI_TalonSRX(1);
    backRight = new WPI_TalonSRX(4);
    
              //IMPORTANT!!
    frontLeft.configPeakCurrentLimit(10, 10); /* 35 A */
    frontLeft.configPeakCurrentDuration(200, 10); /* 200ms */
    frontLeft.configContinuousCurrentLimit(10, 10); 


    frontRight.configPeakCurrentLimit(10, 10); 
    frontRight.configPeakCurrentDuration(200, 10); /* 200ms */
    frontRight.configContinuousCurrentLimit(10, 10); /* 30 */

    backLeft.configPeakCurrentLimit(10, 10); /* 35 A */
    backLeft.configPeakCurrentDuration(200, 10); /* 200ms */
    backLeft.configContinuousCurrentLimit(10, 10); 

    backRight.configPeakCurrentLimit(10, 10); 
    backRight.configPeakCurrentDuration(200, 10); /* 200ms */
    backRight.configContinuousCurrentLimit(10, 10); /* 30 */

    m_drivetrain = new DifferentialDrive(frontLeft, frontRight);

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    
  }
  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public Command getDriveCommand(double forward){
    return this.runOnce(
      () -> {
        //m_drivetrain.feed();
        //arcadeDrive(forward);
        frontLeft.set(-forward);
        frontRight.set(forward);
        backLeft.set(-forward);
        backRight.set(forward);
      }
      );

  }

  public Command getFollow(){
    return this.runOnce (
      () -> {
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
      }
    );
  }

  public Command getDriveContCommand(double forward){
    return this.startEnd(
      () -> {
       // frontLeft.setInverted(true);
        //m_drivetrain.feed();
         frontLeft.setExpiration(0.1);
         frontRight.setExpiration(0.1);
         backLeft.setExpiration(0.1);
         backRight.setExpiration(0.1);
      //   frontLeft.setSafetyEnabled(false);
      //  frontRight.setSafetyEnabled(false);
        frontLeft.set(-forward);
        frontRight.set(forward);
        backLeft.set(-forward);
        backRight.set(forward);
      },
      () -> {
        //m_drivetrain.feed();
        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);
      }
    );
  }

  // public Command getDriveArcade(double speed){
  //   return this.runOnce(
  //     () -> {
  //       m_drivetrain.arcadeDrive(speed, 0);
  //     }
  //   );
  // }



    public Command getTurnCommand(double power){
    return this.runOnce(
      () -> {
        //arcadeDrive(forward);
        frontLeft.set(-power);
        frontRight.set(-power);
        backLeft.set(power);
        backRight.set(power);
      }
      );


  }

  public Command Safety(){
    return this.runOnce(
      () -> {
        frontLeft.setSafetyEnabled(true);
        frontRight.setSafetyEnabled(true);
        backLeft.setSafetyEnabled(true);
        backRight.setSafetyEnabled(true);
        // frontLeft.setExpiration(0.1);
        // frontRight.setExpiration(0.1);
        // backRight.setExpiration(0.1);
        // backLeft.setExpiration(0.1);
        m_drivetrain.feed();
      }
    );
  }

  public Command getTurnContCommand(double power){
    return this.startEnd(
      () -> {
        frontLeft.set(power);
        frontRight.set(-power);
      }, 
      () -> {
        frontLeft.set(0);
        frontRight.set(0);
      }
    );
  }


//     public Command getRotateCommand(double forward, double rotation){
//     return this.runOnce(
//       () -> {
//         arcadeDrive(0, rotation);

//   }
//   );
// }

public void stop(){
  frontLeft.stopMotor();
  frontRight.stopMotor();
}

public Command getStopDriveCommand() {
  return this.startEnd(
      ()-> {
          stop();
      }, 
      ()-> {stop();
});
}


  // @Override
  // public void periodic() {

  //   // SmartDashboard.putNumber("Right Encoder distance", getRightEncoderDistance());
  //   // SmartDashboard.putNumber("Right Encoder velocity", getRightEncoderVelocity());
  //   // SmartDashboard.putNumber("Left Encoder velocity", getLeftEncoderVelocity());
  //   // SmartDashboard.putNumber("Left Encoder distance", getLeftEncoderDistance());
  //   // SmartDashboard.putNumber("Left current", frontLeft.getSupplyCurrent());
  //   // SmartDashboard.putNumber("Right current", frontRight.getSupplyCurrent());
  //   // SmartDashboard.putNumber("Gyro heading", getHeading());
  //   // SmartDashboard.putNumber("Gyro angle", m_navX.getAngle());
  //   // //System.out.println("Test");
  //   /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
  //    * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  // }
}
