// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class EndEffector extends SubsystemBase {
  /** Creates a new endEffector subsystem. */

  // Define the top intake motor, a TalonFX motor, with a unique CAN ID for
  // identification on the CAN bus.
  static TalonFX intake = new TalonFX(Constants.IntakeConstants.IntakeCAN_ID);
  private final SparkMax leftIntakeSparkMax = new SparkMax(IntakeConstants.LEFT_SPARKMAX, SparkMax.MotorType.kBrushless);
  private final SparkMax rightIntakeSparkMax = new SparkMax(IntakeConstants.RIGHT_SPARKMAX, SparkMax.MotorType.kBrushless);
  private SparkMaxConfig intakeMasterSparkMaxConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeFollowerSparkMaxConfig = new SparkMaxConfig();
  // Define the digital inputs for the beam break sensors that are used to detect
  // objects in the intake or discharge area.
  // These sensors are activated when an object crosses the sensor beam.
  static DigitalInput BBIntake = new DigitalInput(Constants.IntakeConstants.BeamBreakPinIntake);
  static DigitalInput BBDis = new DigitalInput(Constants.IntakeConstants.BeamBreakPinDis);

  // Define a DoubleSolenoid to control the movement of the intake tray mechanism.
  // This solenoid can toggle between two positions: forward and reverse.

  // Shuffleboard tab for displaying the status of the intake system's sensors in
  // real-time on the driver station dashboard.

  private ShuffleboardTab DS_EndEffectorTab = Shuffleboard.getTab("EndEffector");

  // Shuffleboard entries to show the real-time state of the beam break sensors,
  // indicating whether an object is present.
  private GenericEntry DS_intakeBeamBreak = DS_EndEffectorTab.add("BB Forward",
      true).getEntry();
  private GenericEntry DS_disBeamBrake = DS_EndEffectorTab.add("BB Back",
      true).getEntry();
  private GenericEntry DS_IntakeSpeed = DS_EndEffectorTab.add("Intake Speed",
      .3)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .getEntry();
  // Constructor for the EndEffector subsystem. Initializes the hardware
  // components and
  // their configurations.

  double intakeSpeed = this.DS_IntakeSpeed.getDouble(0.2);

  public EndEffector() {
    // Configure the top intake motor settings through the motor's configurator.
   configureSparkmax();
  }
  private void configureSparkmax() {
    // Set the idle mode to 'brake', ensuring the motor resists movement when not powered.
    // This helps prevent the wrist from moving unintentionally due to gravity or momentum.
    intakeMasterSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMITS); // Protects motor hardware by limiting current.
    // Apply the configurations to the motor controllers.
    leftIntakeSparkMax.configure(intakeMasterSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeFollowerSparkMaxConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMITS) // Protects motor hardware by limiting current.
        .follow(IntakeConstants.LEFT_SPARKMAX, true ); // Follow the master motor controller.
    rightIntakeSparkMax.configure(intakeFollowerSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}
  // Method to set the power of the top intake motor (TalonFX). The power is
  // passed as a parameter.
  public  void setIntakePower( double power) {
    leftIntakeSparkMax.set(power);// Set the motor to the desired power.
  }

  // Method to control the top intake motor with a specific power. The power is
  // passed as a parameter, and it is reversed.
  public void setIntake(double power) {
    setIntakePower(power); // Set the intake motor to the specified power in reverse.
  }

  // Test method to control the intake motor with conditional power based on the
  // beam break sensor states.
  // If either of the sensors detects an object, it limits the motor power to a
  // lower value.

  public Trigger BBLockout() {
    return new Trigger(() -> this.disBBValue() && !this.intakeBBValue());
    // Create a trigger that activates when the wrist position is below 0.09
  }

  private double intakeBeamStop(double power) {
    double output = 0; // Initialize the output power to 0.

    // If the discharge beam break sensor is not triggered or the intake sensor is
    // not triggered, stop the intake.
    // If the discharge sensor is triggered, allow the intake motor to run at a low
    // power (0.05) to prevent jam.
    if (!this.disBBValue() || !this.intakeBBValue()) {
      if (!this.disBBValue()) {
        output = 0; // Stop the intake motor if the discharge sensor is not triggered.
      } else {
        output = 0.1; // Set a small power if the intake sensor is not triggered.
      }
    } else {
      output = power; // Otherwise, run the intake motor at the specified power.
    }

    return output; // Return the computed power for the intake motor.
  }

  // Method to stop the intake motor by setting its power to 0.
  public void stopIntake() {
    setIntakePower(0); // Stop the intake motor.
  }

  // Method to check the status of the intake beam break sensor.
  // Returns true if the sensor is not triggered (i.e., no object in the intake).
  public boolean intakeBBValue() {
    return BBIntake.get(); // Return the value of the intake beam break sensor.
  }

  // Method to check the status of the discharge beam break sensor.
  // Returns true if the sensor is not triggered (i.e., no object in the discharge
  // area).
  public boolean disBBValue() {
    return BBDis.get(); // Return the value of the discharge beam break sensor.
  }

  // Command to manually run the intake system in the forward direction (intake
  // mode).

  public Command createIntakeCommand(Supplier<Double> powerSupplier) {
    return new FunctionalCommand(
        () -> {
        }, // No initialization needed
        () -> setIntake(powerSupplier.get()), // Set intake motor power dynamically
        interrupted -> stopIntake(), // Stop motor if interrupted
        () -> false, // Runs indefinitely
        this // Pass the subsystem
    );
  }

  // Overload for static power values
  public Command createIntakeCommand(double power) {
    return createIntakeCommand(() -> power);
  }

  // Usage of the reusable method with original names and casing
  // Command to manually run the intake system in the forward direction (intake
  // mode)
  public Command shootCoral() {
    return createIntakeCommand(.5);
  }
  public Command shootCoralSlow() {
    return createIntakeCommand(0.3);
  }


  // Command to manually run the intake system in the reverse direction (backfeed
  public Command manualBackFeed() {
    return createIntakeCommand(-0.05);
  }

  // Command to manually run the intake system in the reverse direction (backfeed)
  public Command IntakeAlgea() {
    return createIntakeCommand(-1);
  }

  public Command HoldAlgea() {
    return createIntakeCommand(-0.2);
  }

  // Command to manually run the intake system in the reverse direction (backfeed)
  public Command ShootAlgea() {
    return createIntakeCommand(.2);
  }

  // Command to stop the intake system
  public Command nothing() {
    return createIntakeCommand(0.0);
  }

  // Dynamic power intake based on sensor input
  public Command IntakeCoral() {
    return createIntakeCommand(() -> intakeBeamStop(.5)).until(() -> !this.disBBValue());
  }

  // Command to toggle the intake solenoid, which controls the intake mechanism's
  // position.
  // This solenoid can switch between forward and reverse positions.

  // Periodic method that is called once per scheduler run.
  // It updates the status of the beam break sensors on the Shuffleboard
  // dashboard.
  @Override
  public void periodic() {
    // Update the state of the beam break sensors on the Shuffleboard dashboard for
    // real-time monitoring.

    this.DS_intakeBeamBreak.setBoolean(intakeBBValue()); // Display the intake sensor status.
    this.DS_disBeamBrake.setBoolean(disBBValue()); // Display the discharge sensor status.
    this.intakeSpeed = this.DS_IntakeSpeed.getDouble(0.2);

  }
}