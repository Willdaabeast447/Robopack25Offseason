// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Import required classes
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Elevatorsp extends SubsystemBase {
  /**
   * Creates a new Elevator subsystem. This is where the elevator motor control
   * and logic reside.
   */
 private class Constants
 {

  public static final int TOP_LIMIT_POSITION = 220;

  public static final double MAX_HEIGHT = 2.5; // meters
  public static final double MIN_HEIGHT = 0; // meters
  public static final double MAX_SPEED  = .5; // meters per second
  public static final double JOYSTICK_POWER_LIMITER = 0.8;
  public static final int ELEVATOR_CURRENT_LIMITS= 50;
  public static final double ELEVATOR_POSITION_TOLERANCE = 0;
  public static final double CORAL_L0 = 0;
  public static final double CORAL_L1 = 10.88;
  public static final double CORAL_L2 = 66.43;
  public static final double CORAL_L3 = 120;
  public static final double CORAL_L4 = 210;
  public static final double ALGEE_L1 = 75.24;
  public static final double ALGEE_L2 = 129.8;
  public static final double PROCESSOR_POSITION = 0.2;
  public static final double Barge = 219;
  public static final double ELEVATOR_CLIMB = 60;

  public static final int ELEVATOR_ERROR_TOLERANCE = 1;
  public static final double HOME_MOTOR_POWER = -0.2;

  
  public static final int LEFT_SPARKMAX = 5;
  public static final int RIGHT_SPARKMAX = 3;
  public static final double INTAKE = 0;

  //IDs
  public static final int LIMIT_SWITCH_CHANNEL = 2;
  public static final int CAN_RANGE_ID = 27;
  public static final int MED_FILTER_ID = 12;
  public static final int LINEAR_FILTER_ID = 12;
  public static final double PID_KP = 0.08;
  public static final double PID_KI = 0;
  public static final double PID_KD = 0;

  // Config <reminder to do>
  
 }
  // Define the TalonFX motor controllers for both sides of the elevator (left and
  // right).
  private final SparkMax elevatorTalonPort = new SparkMax(Constants.LEFT_SPARKMAX, SparkMax.MotorType.kBrushless);
  private final SparkMax elevatorTalonStrb =  new SparkMax(Constants.RIGHT_SPARKMAX, SparkMax.MotorType.kBrushless);
  public RelativeEncoder encoder = elevatorTalonStrb.getEncoder();
  private final DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_CHANNEL);
  public CANrange canRange = new CANrange(Constants.CAN_RANGE_ID);
  public MedianFilter medFilter = new MedianFilter(Constants.MED_FILTER_ID);
  LinearFilter linearFilter = LinearFilter.movingAverage(Constants.LINEAR_FILTER_ID);
  private double filterrange = 0;
  private double calrot=0;
  private final  SparkMaxConfig elevatorConfig = new SparkMaxConfig(); // Create a configuration object for the motor
  // Flag to check if the elevator is allowed to lift based on a condition.
  private boolean canLift = false; // This variable sets whether the elevator can move up or down

  // Create a PID controller to control the elevator position, with initial PID
  // values (Proportional, Integral, Derivative). 

  private final PIDController elevatorController = new PIDController(Constants.PID_KP, Constants.PID_KI, Constants.PID_KD); // The OG value was 0.0002348, but it
                                                                                  // has been adjusted

  // Setup Shuffleboard (WPILib's dashboard) for real-time monitoring of the
  // elevator state during the match.

  private ShuffleboardTab DS_ElevatorTab = Shuffleboard.getTab("Elevator");
  private GenericEntry DS_ElevatorPosition = DS_ElevatorTab.add("ElevatorValue", 0).getEntry(); // Entry for elevator
  // position

  private GenericEntry DS_canLift = DS_ElevatorTab.add("CanLift",
      true).getEntry(); // Entry for canLift

  private GenericEntry DS_ElevatorSetpoint = DS_ElevatorTab.add("Setpoint",
      elevatorController.getSetpoint())
      .getEntry();

  // Default max elevator speed as defined by Shuffleboard.
  // double maxElevatorSpeed = this.DS_ElevatorSpeed.getDouble(0.2);

  public Elevatorsp() {
    // Set both motor controllers to Coast mode to stop the motors from coasting
    // when no power is applied.
    configureSparkmax();

    // Set initial PID controller setpoint to current elevator position.
    elevatorController.setSetpoint(getPosition());
    elevatorController.setTolerance(Constants.ELEVATOR_ERROR_TOLERANCE); // Set tolerance to 1 (tolerance defines when the PID controller considers
    // the
    // target reached)
    // elevatorTalonStrb.setPosition(0); // Possible initial position setting for
    // the second motor (commented out)
  
  }
  private void configureSparkmax() {
    // Set the idle mode to 'brake', ensuring the motor resists movement when not powered.
    // This helps prevent the elevator from moving unintentionally due to gravity or momentum.
    elevatorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.ELEVATOR_CURRENT_LIMITS)
        .inverted(true); // Protects motor hardware by limiting current.
    elevatorConfig.softLimit
    .forwardSoftLimitEnabled(false)
    .reverseSoftLimitEnabled(false);



        

    // Apply the configurations to the motor controllers.
    elevatorTalonPort.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorTalonStrb.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  // Method to check if the elevator has reached its setpoint.
  public boolean elevatorAtSetpoint() {
    return this.elevatorController.atSetpoint(); // Returns true if the elevator is at its setpoint.
  }

  // Trigger that checks if the elevator is in the correct position for intake
  // (position <= 1).

  
  public Trigger elevatorIntake() {
    return new Trigger(() -> this.getPosition() <= 1.14);
  }

  // Method to get the current position of the elevator by reading the position
  // from the right motor.
  public double getPosition() {
    return (encoder.getPosition()); // Return the current position of the elevator.

  }

  /*
   * Setters for the elevator motors.
   */

  // Helper method to set the power (speed) of a given motor.
  private void setMotorPower(SparkMax motor, double power) {
    motor.set(power);
     // Apply the desired power to the motor
  }

  // Method to set power to both elevator motors, considering limits.
  public void setElevatorMotor(double power) {
    double output = elevatorLimit(power);
    double kFValue = kfValueSetter();
    SmartDashboard.putNumber("LimitOutput", output); //
    SmartDashboard.putNumber("Kf value", kFValue);
    // Apply limit on power to prevent the elevator from exceeding boundaries (e.g.,
    // going beyond the upper or lower limit).
    setMotorPower(elevatorTalonPort, output + kFValue); // Apply power to the left motor with limits.
    setMotorPower(elevatorTalonStrb, output + kFValue); // Apply power to the right motor with limits.
    // we added kF value to here to make sure the elevator will hold when it is fed
    // a zero from its limiter

  }
  public boolean atTopLimint() {
    return getPosition() >= Constants.TOP_LIMIT_POSITION;
  
  }

  // Limit the motor power based on certain conditions such as the current
  // position of the elevator and whether it can lift.
  private double elevatorLimit(double power) {
    double output = 0; // Default output is 0 (no power).
    SmartDashboard.putNumber("Input", power);
    // If the elevator can't lift or if the elevator is at the top or bottom, set
    // the output power to a small value.
    if ((getPosition() >= Constants.TOP_LIMIT_POSITION && power > 0) // Positive Power makes the
        // robot go up negative makes the robot go down
        || (getPosition() <= 1 && power < 0)) { //

      output = 0.0;
      // Output is zero but is given a kf value of .02 when it is applied to the motor
      // to allow the elevator to hold.
    } else {
      output = power;
      // Otherwise, apply the requested power to the motors.
    }
    SmartDashboard.putNumber("output", output);
    return output; // Return the calculated output power.
  }

  // Methods for controlling the elevator using PID.
  public void setPID(double setPoint) {

    this.elevatorController.setSetpoint(setPoint); // Tells the PID controller what the setpoint is

  }

  public double kfValueSetter() {
    double kF = 0;
    if (this.getPosition() < 0.5) {
      kF = 0.01;
    } else {
      kF = 0.025;
    }

    return kF;
  }

  public double Throttle() {
    return ((1 - 0.5 * (this.getPosition() / 65.71))); // This method is used to slow the drivespeed down based
                                                               // on
    // the elevator position
    // any slower than this will make slowmode not be able to be held down on the
    // controller or the robot cannot move (OG value for division problem 115 new
    // value is 65.71)
  }

 public Trigger canFold() { // this method sets whether the wrist can fold back based on the elevator
                            // position this prevents folding back into the crossmembers
                            // Original values in order 26, 50.5, 59, 104
                           // new values in order 14.86, 28.86, 33.71, 59.43
  return new Trigger(() -> true);  //(!((this.getElevatorPosition() >= 14.86) && (this.getElevatorPosition() <= 28.86))
       //|| !((this.getElevatorPosition() >= 33.71) && (this.getElevatorPosition() <= 59.43))));

 }

 //

  public void zeroElevator() // Check if the built-in reverse limit
  {
    {
   encoder.setPosition(0);// Reset the value to zero
    }
  }

  // Method to calculate and apply the PID output to move the elevator towards the
  // setpoint.
  public void executeElevatorPID() {
    // Call the PID controller to calculate the required power and apply it to the
    // motors.
    double PIDValue = this.elevatorController.calculate(this.getPosition());

    setElevatorMotor(PIDValue);
  }

  // Command for manual control of the elevator during teleop, allowing the driver
  // to move the elevator with a joystick.
  public Command ManualElevator(DoubleSupplier elevatorJoystick, BooleanSupplier 
  wristLimiter) {
    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean(); // Update whether lifting is allowed based on wrist limiter.
        },

        () -> {
          // Use joystick input to control elevator power. Apply a scaling factor of 0.2
          // for smooth control.
          this.setElevatorMotor(elevatorJoystick.getAsDouble() * Constants.JOYSTICK_POWER_LIMITER );// 0.4 + 0.1
          this.canLift = wristLimiter.getAsBoolean(); // Update lifting condition.
        },

        interrupted -> {this.setPID(this.getPosition());}, // If interrupted, reset the PID setpoint.

        () -> {
          return false; // No specific condition is required for the command to finish.
        },

        this);
  }

  public Command homeCommand() {
    return new FunctionalCommand(
        () -> {
           // Update whether lifting is allowed based on wrist limiter.
        },

        () -> {
          // Use joystick input to control elevator power. Apply a scaling factor of 0.2
          // for smooth control.
          setMotorPower(elevatorTalonPort, Constants.HOME_MOTOR_POWER); // Apply power to the left motor with limits.
          setMotorPower(elevatorTalonStrb, Constants.HOME_MOTOR_POWER); SmartDashboard.putString("complete", "not complete");// 0.4 + 0.18
           // Update lifting condition.
        },

        interrupted -> {setMotorPower(elevatorTalonPort, 0); // Apply power to the left motor with limits.
          setMotorPower(elevatorTalonStrb,0);
          SmartDashboard.putString("complete", "complete");}, // If interrupted, reset the PID setpoint.

        () -> {
          return !limitSwitch.get() ; // No specific condition is required for the command to finish.
        },

        this);
  }

  // A command for starting the elevator with no movement but initializing
  // necessary states.
  public Command startCommand(BooleanSupplier wristLimiter) {
    return this.runOnce(() -> {
      this.canLift = wristLimiter.getAsBoolean(); // Update whether lifting is allowed.
      this.setPID(this.getPosition());
    } // Set the PID setpoint to current position.
    );
  }

 


  // Default PID elevator control command, continuously adjusting the elevator's
  // position based on the PID controller.
  public Command elevatorPIDCommandDefault(BooleanSupplier wristLimiter) {
    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean(); // Update lifting condition.
        },

        () -> {
          this.canLift = wristLimiter.getAsBoolean();
          this.executeElevatorPID(); // Execute PID control to adjust the elevator's position.
        },

        interrupted -> {
          this.canLift = wristLimiter.getAsBoolean(); // Update lifting condition when interrupted.
        },

        () -> {
          return false; // No condition for command termination.
        },

        this);
  }

  // Command to move the elevator to a position based on a given setpoint. 1.09
  // rotations = 1 inch of height
  public Command MovetoPosition(BooleanSupplier wristLimiter, double position) {
    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean(); // Update lifting condition.
          this.setPID(position); // Set PID setpoint to 0 (L1 position).
        },

        () -> {
          this.canLift = wristLimiter.getAsBoolean();
          this.executeElevatorPID(); // Execute PID control to move the elevator.
        },

        interrupted -> {
          this.setPID(this.getPosition());
          this.canLift = wristLimiter.getAsBoolean();
          this.setElevatorMotor(0);
        }, // Nothing new runs when interrupted

        () -> (this.elevatorAtSetpoint()), // Check if the elevator has reached L1.

        this);
  }

  public Command elcal() {
    return new FunctionalCommand(
        () -> {
          // Set PID setpoint to 0 (L1 position).
        },

        () -> {
          // Execute PID control to move the elevator.
        },

        interrupted -> {
          this.setPID(this.calrot);
          encoder.setPosition(this.calrot);
        }, // Nothing new runs when interrupted

        () -> DriverStation.isEnabled(), // Check if the elevator has reached L1.

        this);
  }
  public Command ElevatorL0(BooleanSupplier wristLimiter){

    return MovetoPosition(wristLimiter, Constants.CORAL_L0 );

  }
  public Command ElevatorClimb(BooleanSupplier wristLimiter){

    return MovetoPosition(wristLimiter, Constants.ELEVATOR_CLIMB);

  }
  // Command to move the elevator to the L1 position (0).
  public Command ElevatorL1(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, Constants.CORAL_L1);//0 is orginal

  }

  // Command to move the elevator to the L2 position OG(20) New (11.43)
  public Command ElevatorL2(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, Constants.CORAL_L2);
  }

  // Command to move the elevator to the L3 position OG(52.7). New (30.11)
  public Command ElevatorL3(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, Constants.CORAL_L3);//before (125)
  }

  // Command to move the elevator to the L4 position OG (113.7). New(64.97)
  public Command ElevatorL4(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, Constants.CORAL_L4);//before(207.5)
  }

  public Command ElevatorA1(BooleanSupplier wristLimiter){

    return MovetoPosition(wristLimiter, Constants.ALGEE_L1); // OG (39) New (22.29)
  }

  public Command ElevatorA2(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, Constants.ALGEE_L2); // OG (71) New (40.57)
  }

  public Command ElevatorProcessor(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, Constants.PROCESSOR_POSITION);
  }

  public Command ElevatorBarge(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, 
    Constants.Barge).until(() -> atTopLimint()); // OG
                                                                                                                         // (114.4)
                                                                                                                         // New
                                                                                                                         // (65.37)
  }

  // Command to exit the current elevator state and maintain its position.
  public Command ExitState(BooleanSupplier wristLimiter) {
    return MovetoPosition(wristLimiter, this.getPosition());

  }

  // Periodic method called once per scheduler run to update real-time data on0
  // Shuffleboard for monitoring.
  @Override
  public void periodic() {

    filterrange=linearFilter.calculate(medFilter.calculate(canRange.getDistance().getValueAsDouble()));
    calrot= (Math.pow(filterrange,2) * -19.97 ) +149.98*filterrange + 1.88;
    SmartDashboard.putNumber("canRange Value",canRange.getDistance().getValueAsDouble());
    // checkLimitAndReset();
    SmartDashboard.putNumber("CanRangeElevator", calrot);
    //SmartDashboard.putNumber("CanRange_Linear", (linearFilter.calculate(canRange.getDistance().getValueAsDouble())));
    //SmartDashboard.putNumber("CanRange_median", ( medFilter.calculate(canRange.getDistance().getValueAsDouble())));
   
    // Update the maximum speed value from Shuffleboard.

    // this.maxElevatorSpeed = this.DS_ElevatorSpeed.getDouble(0.2);

    // Update the current position of the elevator.
    this.DS_ElevatorPosition.setDouble(getPosition());
    if ((!limitSwitch.get()) && (this.getPosition() != 0)) {
      this.zeroElevator();
      SmartDashboard.getNumber("elevatorTalonPort speed",elevatorTalonPort.get());
      SmartDashboard.getNumber("elevatorTalonStrb speed",elevatorTalonStrb.get());
    }

    // Update the current status of the forward and reverse limit switches.

    this.DS_ElevatorSetpoint.setDouble(elevatorController.getSetpoint());
    this.DS_canLift.setBoolean(this.canLift);
    SmartDashboard.putData(this);
    SmartDashboard.putBoolean("elevator limit", limitSwitch.get());
    
    // The periodic method is called to regularly update the robot's status.
  }

}
