// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SampleSubsystem extends SubsystemBase {
  private final SparkMax sampleNeo;
  private final SparkFlex sampleFlex;

  private final SparkMaxConfig sampleNeoConfig;
  private final SparkFlexConfig sampleFlexConfig;

  public final RelativeEncoder sampleNeoEncoder;
  public final RelativeEncoder sampleFlexEncoder;
  
  public final PIDController sampleNeoPID = new PIDController(2.0, 0.0, 0.0);
  public final PIDController sampleFlexPID = new PIDController(2.0, 0.0, 0.0);

  private double neoPos = 0.0;
  private double flexPos = 0.0;

  /** Creates a new SampleSubsystem. */
  public SampleSubsystem() {
    sampleNeo = new SparkMax(Constants.SampleConstants.sampleNeo, MotorType.kBrushless);
    sampleFlex = new SparkFlex(Constants.SampleConstants.sampleFlex, MotorType.kBrushless);

    sampleNeoConfig = new SparkMaxConfig();
    sampleFlexConfig = new SparkFlexConfig();

    sampleNeoConfig.inverted(false);
    sampleFlexConfig.inverted(false);

    sampleNeoConfig.idleMode(IdleMode.kBrake);
    sampleNeoConfig.idleMode(IdleMode.kCoast);

    sampleNeoEncoder = sampleNeo.getEncoder();
    sampleFlexEncoder = sampleFlex.getEncoder();

    sampleFlexEncoder.setPosition(0);
    sampleFlexConfig.encoder.positionConversionFactor(1/80.0); // Double check over this.
    // sampleFlexEncoder.setPositionConversionFactor(1/80.0);

    sampleNeo.configure(sampleNeoConfig, ResetMode.kResetSafeParameters, null);
    sampleFlex.configure(sampleFlexConfig, ResetMode.kResetSafeParameters, null);
  }

  public void setPower(double power) {
    sampleNeo.set(power);
    sampleFlex.set(power);
  }

  public void setPosition(double position) {
    neoPos = position;
    flexPos = position;
    
    sampleNeoPID.setSetpoint(neoPos);
    sampleFlexPID.setSetpoint(flexPos);
  }

  public void setNeoPosition(double position) {
    neoPos = position;
    
    sampleNeoPID.setSetpoint(neoPos);
  }

  public void setFlexPosition(double position) {
    flexPos = position;
    
    sampleFlexPID.setSetpoint(flexPos);
  }

  public Command setNeoPositionCommand(double position) {
    return run(() ->  sampleNeoPID.setSetpoint(position));
  }

  public Command setFlexPositionCommand(double position) {
    return run(() ->  sampleFlexPID.setSetpoint(position));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}