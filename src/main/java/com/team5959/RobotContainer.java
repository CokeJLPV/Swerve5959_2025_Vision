// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team5959.Constants.ControllerConstants;
import com.team5959.commands.SwerveDriveJoystickCmd;
import com.team5959.commands.SwerveDriveXLockCmd;
import com.team5959.subsystems.SwerveChassis;
import com.team5959.subsystems.armIntakeAlgaeSubsystem;
import com.team5959.subsystems.elevatorSubsystem;
import com.team5959.subsystems.intakeCoralSubsystem;
import com.team5959.subsystems.miniArmSubsystem;

// Import statements for various WPILib classes and custom classes used in the robot code.
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  public static SendableChooser<Command> autoCommandChooser; //Create sendable chooser for Autos

  // Creacion de objetos de SUBSISTEMAS 
    private final Vision vision;
  private final SwerveChassis swerveChassis;
  /*private final intakeCoralSubsystem intakeCoralSubsystem = new intakeCoralSubsystem();
  private final armIntakeAlgaeSubsystem armIntakeAlgaeSubsystem = new armIntakeAlgaeSubsystem();
  private final elevatorSubsystem elevatorSubsystem = new elevatorSubsystem();
  private final miniArmSubsystem miniArmSubsystem = new miniArmSubsystem();*/

  /*COMMANDS without files
  //Coral Intake Commands
  private Command runInCoralIntakeCommand(){
    return armIntakeAlgaeSubsystem.startEnd(() -> intakeCoralSubsystem.runInCoralIntake(), intakeCoralSubsystem::stopCoralIntake);
  }

  private Command runOutCoralIntakeCommand(){
    return armIntakeAlgaeSubsystem.startEnd(() -> intakeCoralSubsystem.runOutCoralIntake(), intakeCoralSubsystem::stopCoralIntake);
  }

  private Command stopCoralIntakeCommand(){
    return intakeCoralSubsystem.runOnce(() -> intakeCoralSubsystem.stopCoralIntake());
  }
  //Algae Arm Intake Commands
  private Command getRunInAlgaeCommand() {
    return armIntakeAlgaeSubsystem.startEnd(() -> armIntakeAlgaeSubsystem.setAlgaeIntakeSpeed(0.7), armIntakeAlgaeSubsystem::stopAlgaeIntake);
  }

  private Command getRunOutAlgaeCommand() {
    return armIntakeAlgaeSubsystem.startEnd(() -> armIntakeAlgaeSubsystem.setAlgaeIntakeSpeed(-0.5), armIntakeAlgaeSubsystem::stopAlgaeIntake);
  }

  //Algae Arm Position Commands
  private Command getArmPIDMovement() {
    return armIntakeAlgaeSubsystem.run(armIntakeAlgaeSubsystem::runPIDArmTarget).until(armIntakeAlgaeSubsystem::isAtTargetPosition);
  }

  private Command getInOrOutPositionCommand() {
    return Commands.sequence(armIntakeAlgaeSubsystem.runOnce(armIntakeAlgaeSubsystem::inorOutPositionSwitch), getArmPIDMovement());
  }

  private Command getInPerimeterPositionCommand() {
    return Commands.sequence(armIntakeAlgaeSubsystem.runOnce(armIntakeAlgaeSubsystem::moveToInPerimeterPosition), getArmPIDMovement());
  }

  private Command getHoldAlgaeArmPositionCommand() {
    return Commands.sequence(armIntakeAlgaeSubsystem.runOnce(armIntakeAlgaeSubsystem::currentToTargetPosition), armIntakeAlgaeSubsystem.run(armIntakeAlgaeSubsystem::runPIDArmTarget));
  }

  //Elevator Commands
  private Command getElevatorPIDMovement() {
    return elevatorSubsystem.run(elevatorSubsystem::runPIDElevatorTarget).until(elevatorSubsystem::atTargetPosition);
  }

  private Command getHoldElevatorPositionCommand() {
    return Commands.sequence(elevatorSubsystem.runOnce(elevatorSubsystem::CurrentToTargetPosition), elevatorSubsystem.run(elevatorSubsystem::runPIDElevatorTarget));
  }

  private Command getElevatorMoveCommand(Runnable positionSetter) {
    return Commands.sequence(elevatorSubsystem.runOnce(positionSetter), getElevatorPIDMovement());
  }

  //Mini Arm Commands
  private Command getMiniArmPIDMovement() {
    return miniArmSubsystem.run(miniArmSubsystem::runPIDMiniArmTarget).until(miniArmSubsystem::atTargetPosition);
  }

  private Command getHoldMiniArmPositionCommand() {
    return Commands.sequence(miniArmSubsystem.runOnce(miniArmSubsystem::currentToTargetPosition), miniArmSubsystem.run(miniArmSubsystem::runPIDMiniArmTarget));
  }

  private Command scoreReffCommandSequence(){
    return Commands.sequence(runInCoralIntakeCommand().withTimeout(1.5));
  }

  private Command waitToCoral(){
    return intakeCoralSubsystem.run(intakeCoralSubsystem::runInCoralIntake).until(elevatorSubsystem::canRangeDetectsObject).andThen(runInCoralIntakeCommand().withTimeout(0.8));
  }*/

  // Creacion de objetos de CONTROLES
  private final PS4Controller control = new PS4Controller(ControllerConstants.kDriverControllerPort);
  private final CommandPS4Controller CommandPS4Controller = new CommandPS4Controller(ControllerConstants.kDriverControllerPort);
  //private final CommandGenericHID CommandGenericController = new CommandGenericHID(ControllerConstants.kOperatorControllerPort);


  // Creacion de objetos de BOTONES para asignar nombres claros 
  private final JoystickButton resetPosButton = new JoystickButton(control, 9);
  private final JoystickButton resetNavxButton = new JoystickButton(control, 10);
  private final JoystickButton lockPositionButton = new JoystickButton(control, 14);
 

  
   
  public RobotContainer() {
    /* 
    //REGISTER NAME AUTONOMOUS COMMANDS
    NamedCommands.registerCommand("outCoral", runOutCoralIntakeCommand().withTimeout(2));
    NamedCommands.registerCommand("algaeDown", getInOrOutPositionCommand());
    NamedCommands.registerCommand("getAlgae", getRunInAlgaeCommand().withTimeout(2));
    NamedCommands.registerCommand("algaeUp", getInOrOutPositionCommand());
    NamedCommands.registerCommand("score",runInCoralIntakeCommand().withTimeout(1.5));
    NamedCommands.registerCommand("waitToCoral", waitToCoral());*/
    swerveChassis = new SwerveChassis();
    vision = new Vision(swerveChassis::addVisionMeasurement);
    
    autoCommandChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Command Chooser", autoCommandChooser);

    // Build an auto chooser. This will use Commands.none() as the default option.
   // autoChooser = AutoBuilder.buildAutoChooser();

   // For convenience a programmer could change this when going to competition.
   boolean isCompetition = true;

   // Build an auto chooser. This will use Commands.none() as the default option.
   // As an example, this will only show autos that start with "comp" while at
   // competition as defined by the programmer
   autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
     (stream) -> isCompetition
       ? stream.filter(auto -> auto.getName().startsWith("comp"))
       : stream
   );

   SmartDashboard.putData("Auto Chooser", autoChooser);
   SmartDashboard.putData("Command Scheduler", edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance());
  

    

    // Configurar los comandos predeterminados de los subsistemas. En este caso, el chasis swerve
       swerveChassis.setDefaultCommand(new SwerveDriveJoystickCmd(swerveChassis,
        () -> control.getLeftY(), 
        () -> control.getLeftX(), 
        () -> control.getRightX(),
        true));
    /*intakeCoralSubsystem.setDefaultCommand(stopCoralIntakeCommand());
    armIntakeAlgaeSubsystem.setDefaultCommand(getHoldAlgaeArmPositionCommand());
    elevatorSubsystem.setDefaultCommand(getHoldElevatorPositionCommand());
    miniArmSubsystem.setDefaultCommand(getHoldMiniArmPositionCommand());
    */

    //Comandos de ejemplo para usar con PathPlanner
    NamedCommands.registerCommand("RunIntakeCmd", Commands.none());
    NamedCommands.registerCommand("OuttakeCmd", Commands.none());
   
       // Configure the trigger bindings method.
    configureBindings();

    //SmartDashboard.putData("VisionSubsystem",VisionSubsystem);
  }

  // Configurar los enlaces de botones para los comandos usando lambdas o referencias de método
  private void configureBindings() {

    resetNavxButton.onTrue(new InstantCommand(() -> {swerveChassis.resetNavx();swerveChassis.resetHeadingHoldAfterGyroReset();}));
    resetPosButton.onTrue(new InstantCommand(() -> {
      // 1. Resetear navX primero
      //swerveChassis.resetNavx();
      swerveChassis.resetHeadingHoldAfterGyroReset();
  
      // 2. Ahora que el gyro está a 0, usar esa rotación para odometría
      swerveChassis.resetOdometry(new Pose2d(0, 0, swerveChassis.getRotation2d()));
  
      // 3. Resetear encoders de los módulos
      swerveChassis.resetDriveEncoders();
  }, swerveChassis));
  
    lockPositionButton.whileTrue(new SwerveDriveXLockCmd(swerveChassis));
    
    /*CommandPS4Controller.R2().whileTrue(getRunInAlgaeCommand());
    CommandPS4Controller.L2().whileTrue(getRunOutAlgaeCommand());

    CommandPS4Controller.triangle().onTrue(Commands.sequence(miniArmSubsystem.runOnce(miniArmSubsystem::downOrStartingPositionSwitch), getMiniArmPIDMovement())); //Triangle
    CommandPS4Controller.circle().onTrue(Commands.sequence(miniArmSubsystem.runOnce(miniArmSubsystem::moveToDropAlgaePosition), getMiniArmPIDMovement())); //Circle

    CommandPS4Controller.square().onTrue(getInOrOutPositionCommand()); //Square
    CommandPS4Controller.cross().onTrue(getInPerimeterPositionCommand()); //Cross


    CommandGenericController.button(5).whileTrue(elevatorSubsystem.run(elevatorSubsystem::elevatorUpManualMode)); //LB
    CommandGenericController.button(6).whileTrue(elevatorSubsystem.run(elevatorSubsystem::elevatorDownManualMode)); //RB

    CommandGenericController.button(8).whileTrue(runOutCoralIntakeCommand());//LT
    CommandGenericController.button(7).whileTrue(runInCoralIntakeCommand());//RT

    Trigger coralButtonsPressed = CommandGenericController.button(8).or(CommandGenericController.button(7));
    coralButtonsPressed.negate().onTrue(stopCoralIntakeCommand());

    CommandGenericController.button(3).onTrue(getElevatorMoveCommand(elevatorSubsystem::moveToL1Position)); //X
    CommandGenericController.button(4).onTrue(getElevatorMoveCommand(elevatorSubsystem::moveToL2Position)); //Y
    CommandGenericController.button(2).onTrue(getElevatorMoveCommand(elevatorSubsystem::moveToL3Position)); //B
    CommandGenericController.button(1).onTrue(getElevatorMoveCommand(elevatorSubsystem::moveToStartingPosition)); //A
    */

  
  }
  
  public void periodic(){
    vision.periodic();
        
  }
  
  public Command getAutonomousCommand() {
   // return new PathPlannerAuto("Auto1");   

   return autoCommandChooser.getSelected();
       // return new AutoFollowTrajectoryCmd(swerveChassis);
    
  }
}
