// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.MergeVisionOdometryCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevatorsp;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wristsp;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public static class OperatorConstants
  {
     // Joystick Deadband
    public static final double DEADBAND        = 0.2;
    public static final double LEFT_Y_DEADBAND = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.15;
    public static final double TURN_CONSTANT    = 6;
  }


private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
final  CommandXboxController joystick = new CommandXboxController(0);
final  CommandXboxController driverXbox2 =  new CommandXboxController(1);
final  CommandJoystick buttonbord  = new CommandJoystick(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
   // Elevator elevator = new Elevator();
  //final Command elevatorManual = elevator.manualControl(() -> -MathUtil.applyDeadband(joystick2.getLeftY(),
  //OperatorConstants.LEFT_Y_DEADBAND));
 Elevatorsp elevator = new Elevatorsp();
    //Wrist wrist = new Wrist();
  //final Command Wristmanual = wrist.manualControl(() -> -MathUtil.applyDeadband(joystick2.getRightY(),
  //OperatorConstants.LEFT_Y_DEADBAND));
Wristsp wrist = new Wristsp();
  //Intake intake = new Intake();
//private final Trigger HaveCoral = intake.haveCoralTrigger();
 EndEffector endEffector = new EndEffector();
  Climber climber = new Climber();
  Vision vision = new Vision();
  
  private final Trigger wristLimiter;
  private final Trigger canFold;
  private final Trigger wristIntake;
  private final Trigger elevatorIntake;
  private final Trigger isEnabled;
  private final Trigger isDisabled;
  private final Trigger algeaModeEnabled;
  private final Trigger climberlimit;
  private final Trigger sloShoot;
  //private final Trigger reverseLimitHit;
  private final Trigger BBLockout;

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> (-MathUtil.applyDeadband(joystick.getLeftY()*throttle(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND)),
                                                                 () -> (-MathUtil.applyDeadband(joystick.getLeftX()*throttle(),
                                                                                               OperatorConstants.DEADBAND)),
                                                                 () -> (-MathUtil.applyDeadband(joystick.getRightX()*throttle(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND)),
                                                                 joystick.getHID()::getYButtonPressed,
                                                                 joystick.getHID()::getAButtonPressed,
                                                                 joystick.getHID()::getXButtonPressed,
                                                                 joystick.getHID()::getBButtonPressed);


 /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = 
                    SwerveInputStream.of(drivebase.getSwerveDrive(),
                                          () -> -joystick.getLeftY()*throttle(),
                                          () -> -joystick.getLeftX()*throttle())
                                      .withControllerRotationAxis(
                                          ()->-joystick.getRightX()*throttle())
                                      .deadband(OperatorConstants.DEADBAND)
                                      .scaleTranslation(1)
                                      .allianceRelativeControl(true);

                                                             /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(joystick::getRightX,
    joystick::getRightY)
    .headingWhile(true);
 // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  //SwerveInputStream driveRobotRelative = driveAngularVelocity.copy().robotRelative(true);

  Command driveRobotRelative =  drivebase.driveRobotOrientedCommand(
    () ->  (MathUtil.applyDeadband(-joystick.getLeftY(),.1)*throttle())
  , () -> (MathUtil.applyDeadband(-joystick.getLeftX(),.1)*throttle()),
    ()-> (MathUtil.applyDeadband(-joystick.getRightX(),.1))*throttle());
    
  
 
  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -joystick.getLeftY(),
                                                                   () -> -joystick.getLeftX())
                                                               .withControllerRotationAxis(() -> joystick.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
    .withControllerHeadingAxis(() -> Math.sin(
                                 joystick.getRawAxis(
                                     2) * Math.PI) * (Math.PI * 2),
                             () -> Math.cos(
                                 joystick.getRawAxis(
                                     2) * Math.PI) *
                                   (Math.PI * 2))
    .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);
    private boolean AlgeaMode;
  
      /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
      wristLimiter = wrist.wristLimiter();
              
                canFold = new Trigger(()->true);
                BBLockout = endEffector.BBLockout();
                wristIntake = wrist.wristIntake();
                elevatorIntake = elevator.elevatorIntake();
                isEnabled = new Trigger(() -> DriverStation.isEnabled());
                isDisabled = new Trigger(() -> DriverStation.isDisabled());
                algeaModeEnabled = new Trigger(() -> getAlgeaMode());
                climberlimit = new Trigger(() -> climber.getLimitSwitch());
                sloShoot=new Trigger(()->elevator.getPosition()<20);
               // reverseLimitHit = ()-> false;//elevator.reverseLimitHit();

      // Configure the trigger bindings
      configureBindings();
      DriverStation.silenceJoystickConnectionWarning(true);
      NamedCommands.registerCommand("test", Commands.print("I EXIST"));
      NamedCommands.registerCommand("autostart", autoStart());
      NamedCommands.registerCommand("L1_shoot",endEffector.shootCoralSlow().until(()->endEffector.disBBValue()));
      NamedCommands.registerCommand("L1_position",autoL1_position());
      NamedCommands.registerCommand("aVCal",aVCal());// finds elevator position using canRange
      NamedCommands.registerCommand("WristSafety",wrist.WristSafety(() -> canFold.getAsBoolean()));
      NamedCommands.registerCommand("Default position",elevator.ElevatorL2(wristLimiter));//Where the elevator will be at while moving around
      NamedCommands.registerCommand("elcal",elevator.elcal().withTimeout(.5));
      NamedCommands.registerCommand("L1",elevator.ElevatorL1(wristLimiter));
      NamedCommands.registerCommand("A1",Commands.parallel( elevator.ElevatorA1(wristLimiter), wrist.WristA1(() -> canFold.getAsBoolean())));
      NamedCommands.registerCommand("L2",Commands.parallel( elevator.ElevatorL2(wristLimiter), wrist.WristL2(() -> canFold.getAsBoolean())));
      NamedCommands.registerCommand("Shoot_",endEffector.shootCoral().until(()->endEffector.disBBValue()));
      NamedCommands.registerCommand("LoadLevel",Commands.sequence(Commands.parallel(elevator.ElevatorL0(wristLimiter), wrist.WristL0(() -> canFold.getAsBoolean())),endEffector.IntakeCoral().withTimeout(5))); 
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
      NamedCommands.registerCommand("L4",Commands.parallel( elevator.ElevatorL4(wristLimiter), wrist.WristL4(() -> canFold.getAsBoolean())));
      
   
    }

    /**
     * uses the left trigger to control the throttle of the robot
     * @return percentage of throttle from 0 to 50%
     */
    //TODO add correct function
    public void setCanRange()
    {
      elevator.encoder.setPosition(elevator.canRange.getDistance().getValueAsDouble() * 133.33 -1.4);// change 1 to correct value
    }
  
    public double throttle()
    {
      return 1-joystick.getLeftTriggerAxis()*0.5;
    }

    /**
     * Smooths the axis input to make the robot easier to control
     * @param axis the axis to smooth
     * @return the smoothed axis
     */
    public double smoothAxis(double axis)
    {
      return Math.pow(MathUtil.applyDeadband(axis, 0.1)*throttle(),3);
    }

    /**
     * Gets the current state of the algea mode
     * @return true if the robot is in algea mode
     */
    public boolean getAlgeaMode() { // Sets whether the commands are based on algea or coral
      return this.AlgeaMode;

    }

    /**
     * Creates a command that will run the robot through a series of commands to start the robot auto routine to home the elevator with the wrist folded out 
     * @return
     */
    public Command aVCal()
    {
      return Commands.runOnce(()->setCanRange(),elevator);
    }


    public Command autoStart ()
    {
      return elevator.ManualElevator(() -> 0.4,
      () -> wristLimiter.getAsBoolean()).withTimeout(.5).andThen(elevator.ManualElevator(() -> 0,
      () -> wristLimiter.getAsBoolean()).withTimeout(.1))                           
      .andThen(wrist.WristL1(canFold)).andThen(elevator.homeCommand());
      
    }

    public Command autoL1_position()
    {
      return elevator.ElevatorL1(wristLimiter).andThen(wrist.WristL1(canFold));
      
    }
   
    private void configureBindings()
    {

      vision.setDefaultCommand(new MergeVisionOdometryCommand(vision,drivebase));
      
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      joystick.y().toggleOnTrue(driveRobotRelative);
      climber.setDefaultCommand(climber.Stop());
  
      //isDisabled.whileTrue(elevator.elcal().ignoringDisable(true));
      driverXbox2.a().onTrue(elevator.homeCommand());
      

      joystick.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      joystick.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
     
      //joystick.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    
      /* climberlimit.negate().and(joystick.rightBumper().and(joystick.leftBumper()))
        .whileTrue(climber.manualClimber( ()->buttonbord.povUp().getAsBoolean()?0.1:0,
                                          ()->buttonbord.povDown().getAsBoolean()?0.0:0));
      climberlimit.and(joystick.rightBumper().and(joystick.leftBumper())).and(buttonbord.povDownLeft().negate().and(buttonbord.povDownRight().negate()))
        .whileTrue(climber.manualClimber( ()->buttonbord.povUp().getAsBoolean()?0.1:0,
                                          ()->buttonbord.povDown().getAsBoolean()?0.1:0));
      climberlimit.and(joystick.rightBumper().and(joystick.leftBumper())).and(buttonbord.povDownLeft().and(buttonbord.povDownRight().negate()))
      .whileTrue(climber.manualClimber( ()->buttonbord.povUp().getAsBoolean()?0.3:0,
                                        ()->buttonbord.povDownLeft().getAsBoolean()?0.3:0));   
      climberlimit.and(joystick.rightBumper().and(joystick.leftBumper())).and(buttonbord.povDownLeft().negate().and(buttonbord.povDownRight()))
      .whileTrue(climber.manualClimber( ()->buttonbord.povUp().getAsBoolean()?0.6:0, 
                                        ()->buttonbord.povDownRight().getAsBoolean()?0.6:0)); */
    
      joystick.rightBumper().and(joystick.leftBumper())
          .whileTrue(climber.fullForward(
              () -> buttonbord.povDown().getAsBoolean() ? -0.4 : 0));
      
      buttonbord.button(13).onTrue(Commands.sequence(wrist.WristSafety(
        () -> canFold.getAsBoolean()), elevator.ElevatorL4(wristLimiter),
        elevator.ElevatorClimb(wristLimiter),wrist.homeCommand()));
      

      buttonbord.button(11).onTrue(Commands.runOnce(() -> this.AlgeaMode = !this.AlgeaMode));


        // Endeffector command bindings, such as when to turn on intake or control trays
        endEffector.setDefaultCommand(endEffector.nothing()); // Default is do nothing
        // Coral Commands
     /*    algeaModeEnabled.negate().and(elevator.elevatorIntake().and(wrist.wristIntake()))
                        .and(() -> RobotState.isTeleop())
                        .whileTrue(endEffector.IntakeCoral());// When algea mode is disabled and the elevator
                        */                                       // and wrist are in the L1 position

        // intake coral

        algeaModeEnabled.negate().and(buttonbord.button(6)).whileTrue(endEffector.IntakeCoral());
        // When algea mode is diabled and button 5 is hit Intake coral manually
        sloShoot.negate().and(algeaModeEnabled.negate()).and(buttonbord.button(5)).whileTrue(endEffector.shootCoral());
        sloShoot.and(algeaModeEnabled.negate()).and(buttonbord.button(5)).whileTrue(endEffector.shootCoralSlow());
        // When algea mode is diabled and button 2 is hit shoot coral
        algeaModeEnabled.negate().and(buttonbord.button(9)).whileTrue(endEffector.manualBackFeed());
        // When algea mode is diabled and button 3 is hit backfeed coral

        // Algea Commands
        algeaModeEnabled.and(buttonbord.button(6)).whileTrue(endEffector.IntakeAlgea());
        // When algea mode is enabled and button 5 is hit Intake algea
        algeaModeEnabled.and(buttonbord.button(6).negate()).and(buttonbord.button(5)
                        .negate())
                        .whileTrue(endEffector.HoldAlgea());
        // When algea mode is enabled and button 2 and button 5 are not hit hold algea
        // by applying a 3% back spin

        algeaModeEnabled.and(buttonbord.button(5)).and(buttonbord.button(6).negate())
                        .onTrue(endEffector.ShootAlgea().withTimeout(5));
        // When algea mode is enabled and button 2 is hit and button 5 is not hit shoot
        // algea

        // Commands for wrist and elevator control using buttonboard inputs
        wrist.setDefaultCommand(wrist.WristPIDCommandDefault(() -> canFold.getAsBoolean()));
        //wrist.setDefaultCommand(wrist.ManualWrist(() ->- driverXbox2.getRawAxis(5),
        //() -> canFold.getAsBoolean()));
        driverXbox2.axisGreaterThan(5, 0.1).or(driverXbox2.axisLessThan(5, -0.1))
                    .whileTrue(wrist.ManualWrist(() -> -driverXbox2.getRawAxis(5),
                                        () -> canFold.getAsBoolean()));

        // Elevator control with wrist limit consideration
         elevator.setDefaultCommand((elevator.elevatorPIDCommandDefault(() -> wristLimiter.getAsBoolean())));
        /*elevator.setDefaultCommand(elevator.ManualElevator(() -> MathUtil.applyDeadband(-driverXbox2.getRawAxis(1),0.15),
        () -> wristLimiter.getAsBoolean())); */
        (wristLimiter).and((driverXbox2.axisGreaterThan(1, 0.1).or(driverXbox2.axisLessThan(1, -0.1)))
                        .whileTrue(elevator.ManualElevator(() -> -driverXbox2.getRawAxis(1),
                                       () -> wristLimiter.getAsBoolean())));

        // Commands to preserve position when enabled
        //this.isEnabled.onTrue(elevator.startCommand(wristLimiter));
        //this.isEnabled.onTrue(wrist.startWristCommand());
        // this.isDisabled.onTrue(elevator.EndCommand(wristLimiter));

        // Wrist and elevator commands for specific positions, triggered by button
        // presses

        //L4
        (BBLockout.negate()).and(algeaModeEnabled.negate().and(buttonbord.button(12)))
                        .onTrue(Commands.sequence( wrist.WristSafety(
                          () -> canFold.getAsBoolean()),elevator.ElevatorL4(wristLimiter),
                                        wrist.WristL4(() -> canFold.getAsBoolean())));
        //L3
        (BBLockout.negate()).and(algeaModeEnabled.negate().and(buttonbord.button(2)))
                        .onTrue(Commands.sequence( elevator.ElevatorL3(wristLimiter),
                                        wrist.WristL3(() -> canFold.getAsBoolean())));

        //L2
        (BBLockout.negate()).and(algeaModeEnabled.negate().and(buttonbord.button(3)))
                        .onTrue(Commands.sequence( elevator.ElevatorL2(wristLimiter),
                                        wrist.WristL2(() -> canFold.getAsBoolean())));

        //L1                                
        (BBLockout.negate()).and(algeaModeEnabled.negate().and(buttonbord.button(4)))
                        .onTrue(Commands.sequence(elevator.ElevatorL1(wristLimiter),
                                        wrist.WristL1(() -> canFold.getAsBoolean())));
        //Intake level
        (BBLockout.negate()).and(algeaModeEnabled.negate()).and(buttonbord.button(1))
        .onTrue(Commands.sequence(wrist.WristSafety(
          () -> canFold.getAsBoolean()), elevator.ElevatorL0(wristLimiter),
                        wrist.WristL0(() -> canFold.getAsBoolean())));                                
        // Algea Positions//
        (BBLockout.negate()).and(algeaModeEnabled.and(buttonbord.button(3)))
                        .onTrue(Commands.sequence( elevator.ElevatorA1(wristLimiter),
                                        wrist.WristA1(() -> canFold.getAsBoolean())));

        (BBLockout.negate()).and(algeaModeEnabled.and(buttonbord.button(2)))
                        .onTrue(Commands.sequence( elevator.ElevatorA2(wristLimiter),
                                        wrist.WristA2(() -> canFold
                                                        .getAsBoolean())));

        (BBLockout.negate()).and(algeaModeEnabled.and(buttonbord.button(12)))
                        .onTrue(Commands.sequence( wrist.WristSafety(
                          () -> canFold.getAsBoolean()),elevator.ElevatorBarge(wristLimiter),
                                        wrist.WristBarge(() -> canFold.getAsBoolean())));

        (BBLockout.negate()).and(algeaModeEnabled.and(buttonbord.button(4)))
                        .onTrue(Commands.sequence(
                                        elevator.ElevatorProcessor(wristLimiter),
                                        wrist.WristProcessor(() -> canFold
                                                        .getAsBoolean())));
        //Intake level
        (BBLockout.negate()).and(algeaModeEnabled).and(buttonbord.button(1))
                        .onTrue(Commands.sequence( wrist.WristSafety(
                          () -> canFold.getAsBoolean()),elevator.ElevatorL0(wristLimiter),
                                        wrist.WristAlgeaIntake(() -> canFold.getAsBoolean())));
    

 
 
        }
    
   
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    return autoChooser.getSelected();


    //return autoStart();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
