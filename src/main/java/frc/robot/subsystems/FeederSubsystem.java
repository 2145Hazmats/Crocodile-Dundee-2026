// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FeederSubsystem extends SubsystemBase {
  private TalonFX feederMotor;
  private CommandSwerveDrivetrain m_drivetrain;
  
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem(CommandSwerveDrivetrain drivetrain) {
    feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID);
    m_drivetrain = drivetrain;


    var feederCurrentLimitsConfigs = new CurrentLimitsConfigs();
   feederCurrentLimitsConfigs
    .withStatorCurrentLimit(60)
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentLimitEnable(true);
    
    feederMotor.getConfigurator().apply(feederCurrentLimitsConfigs);
    
    
  }

 

  @Override
  public void periodic() {

    
    // This method will be called once per scheduler run
    }
  // Sets the feeder motor to a speed
  public void setFeederMotor(double speed) {
    feederMotor.set(speed);
  }
}
