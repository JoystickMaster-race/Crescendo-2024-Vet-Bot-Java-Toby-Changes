// Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends PIDSubsystem {
  WPI_VictorSPX m_shooterMotor;
  Encoder m_shooterEncoder;
  SimpleMotorFeedforward m_shooterFeedForward;
  NetworkTableInstance table = NetworkTableInstance.getDefault();

public Shooter(){
  super(new PIDController(0, 0, 0));
  m_shooterMotor = new WPI_VictorSPX(kShooterID);
  m_shooterEncoder = new Encoder(4, 5);
  m_shooterFeedForward = new SimpleMotorFeedforward(0.05, kVVoltSecondsPerRotation);
  getController().setTolerance(kShooterTargetRPS);
  m_shooterEncoder.setDistancePerPulse(0);
  setSetpoint(kShooterTargetRPS);
}

public Command getShootCommand() {
  return this.startEnd(
      ()-> {
          setShooterSpeed(kShooterSpeed);
      },
      
      ()-> {
          stop();
      });
}

@Override
public void useOutput(double output, double setpoint){
  m_shooterMotor.setVoltage(output + m_shooterFeedForward.calculate(RadiansPerSecond.of(kShooterTargetRPS).in(RadiansPerSecond)));
}

@Override
public double getMeasurement(){
  return m_shooterEncoder.getRate();
}

public void setShooterSpeed(double speed){
  m_shooterMotor.set(speed);
}

public void stop(){
  m_shooterMotor.set(0);
}

@Override
public void periodic() {
  SmartDashboard.putNumber("Shooter rate", m_shooterEncoder.getRate());
  table.getEntry("rate").setDouble(m_shooterEncoder.getRate());
  table.getEntry("setpoint").setDouble(kShooterFreeRPS);
}

}