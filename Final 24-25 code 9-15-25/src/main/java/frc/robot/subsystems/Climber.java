// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final SparkMax sparkmax = new SparkMax(ClimberConstants.SPARKMAX, SparkMax.MotorType.kBrushless);
  private RelativeEncoder encoder = sparkmax.getEncoder();
   /** Creates a new Climber. */
  private final DigitalInput limitSwitch = new DigitalInput(ClimberConstants.LIMIT_SWITCH);
  private SparkMaxConfig climberSparkMaxConfig=new SparkMaxConfig();
   public Climber() {
    configureSparkmax();
   }

   private void configureSparkmax() {
    // Set the idle mode to 'brake', ensuring the motor resists movement when not powered.
    // This helps prevent the wrist from moving unintentionally due to gravity or momentum.
    climberSparkMaxConfig
        .idleMode(IdleMode.kBrake);
        //.smartCurrentLimit(WristConstants.WRIST_CURRENT_LIMITS); // Protects motor hardware by limiting current.

    // Configure encoder conversion factors to ensure accurate position and velocity tracking.
   climberSparkMaxConfig.softLimit
   .reverseSoftLimit(-390)
   .reverseSoftLimitEnabled(true)
   .forwardSoftLimit(1)
   .forwardSoftLimitEnabled(true);

    // Configure closed-loop control parameters for precise position and velocity management.
   
   

;   // Apply the configurations to the motor controllers.
    sparkmax.configure(climberSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

   private void stop()
   {
    sparkmax.set(0);
   }

   public boolean getLimitSwitch()
   {
     return limitSwitch.get();
   }
   
   public Command Stop() {
    return new FunctionalCommand(
      () -> stop(),
      () -> stop(),
      (interrupted) -> {},
      () -> false,
      this
    );
   }

   public Command OnClimb(DoubleSupplier speed)
   {
     return new FunctionalCommand(
      () -> sparkmax.set(speed.getAsDouble()),
      () -> sparkmax.set(speed.getAsDouble()),
      (interrupted) -> stop(),
      () -> false,
      this                       );
   }

   public Command OffClimb(DoubleSupplier speed)
   {
     return new FunctionalCommand(
      () -> sparkmax.set(-speed.getAsDouble()),
      () ->  sparkmax.set(-speed.getAsDouble()),
      (interrupted) -> stop(),
      () -> false,
      this                       );
   }
   public Command manualClimber(DoubleSupplier inSpeed, DoubleSupplier outSpeed)
   {
     return new FunctionalCommand(
      () -> sparkmax.set(inSpeed.getAsDouble() - outSpeed.getAsDouble()),
      () -> sparkmax.set(inSpeed.getAsDouble() - outSpeed.getAsDouble()),
      (interrupted) -> stop(),
      () -> false,
      this                       );
   }

   public Command fullForward(DoubleSupplier inSpeed)
   {
     return new FunctionalCommand(
      () -> sparkmax.set(inSpeed.getAsDouble() ),
      () ->{ if(encoder.getPosition()<80)
      {sparkmax.set(inSpeed.getAsDouble());}
      else if (encoder.getPosition()<128)
      {
        sparkmax.set(inSpeed.getAsDouble());
      }
      else if (encoder.getPosition()<160)
      {
        sparkmax.set(inSpeed.getAsDouble());
      }
      else if (encoder.getPosition()>160)
      {
        sparkmax.set(inSpeed.getAsDouble()*3);
      }},
      (interrupted) -> stop(),
      () -> false,
      this                       );
   }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("climb limit",limitSwitch.get());
    SmartDashboard.putNumber("climber position",encoder.getPosition());
  }
}
