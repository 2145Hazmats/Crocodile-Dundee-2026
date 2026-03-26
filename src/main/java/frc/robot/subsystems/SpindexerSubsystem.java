// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
  // Provides a name for the KrakenMotor
  private final TalonFX spindexerMotor;
  /** Creates a new SpindexerSubsystem. */
  public SpindexerSubsystem() {
    /** Creates a new TalonFX with MotorID using the name we gave the motor from earlier */
    spindexerMotor = new TalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID);

    var spindexerCurrentLimitsConfigs = new CurrentLimitsConfigs();
   spindexerCurrentLimitsConfigs
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(60)
    .withStatorCurrentLimitEnable(true);
   spindexerMotor.getConfigurator().apply(spindexerCurrentLimitsConfigs);

  }
  
  /** Command that will be run in robot container, it sets the motor at a given speed, */
  // Double needed because it is a decimal value * 100 for set value in percent motor speed, ex: 0.5 * 100 = 50% motor speed
  public void SetMotor(double speed){
    spindexerMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
