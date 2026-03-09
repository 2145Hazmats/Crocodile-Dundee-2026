// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // Creates new motors
  private TalonFX shooterMotor; 
  private TalonFX kickerMotor;
  private TalonFX hoodMotor;
  private final VelocityVoltage m_flywheelRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage m_hoodRequest = new PositionVoltage(0).withSlot(1);
  
  

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
   shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);
   kickerMotor = new TalonFX(ShooterConstants.KICKER_MOTOR_ID);
   hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
   var slot0Configs = new Slot0Configs();
   slot0Configs.kS = 0;
   slot0Configs.kV = 0;
   slot0Configs.kP = ShooterConstants.FLYWHEEL_P;
   slot0Configs.kI = ShooterConstants.FLYWHEEL_I;
   slot0Configs.kD = ShooterConstants.FLYWHEEL_D;
   shooterMotor.getConfigurator().apply(slot0Configs);
  
   var slot1Configs = new Slot1Configs();
   slot1Configs.kP = ShooterConstants.HOOD_P;
   slot1Configs.kI = ShooterConstants.HOOD_I;
   slot1Configs.kD = ShooterConstants.HOOD_D;
   hoodMotor.getConfigurator().apply(slot1Configs);
  
 }

  // Sets the shooter motor to a speed
  public void setShooterMotor(double speed) {
    shooterMotor.set(speed);
  }

  // Sets the kicker motor to a speed
  public void setKickerMotor(double speed) {
    kickerMotor.set(speed);
  }

  public void setHoodMotor(double speed) {
    hoodMotor.set(speed);
  }

  public void setFlywheelToSpeed(double RPM){
    shooterMotor.setControl(m_flywheelRequest.withVelocity(MathConstants.RPMtoRPS(RPM))
    .withFeedForward(0.5));
  }

  public void setHoodMotorPosition(double angle) {
    hoodMotor.setControl(m_hoodRequest.withPosition(MathConstants.DegreesToRotations(angle)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
