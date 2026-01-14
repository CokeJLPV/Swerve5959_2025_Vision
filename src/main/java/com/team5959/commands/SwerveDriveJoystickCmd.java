package com.team5959.commands;



import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import com.team5959.Constants.SwerveConstants;
import com.team5959.subsystems.SwerveChassis;

    public class SwerveDriveJoystickCmd extends Command{
    private SwerveChassis swerveChassis; 

    private DoubleSupplier xSupplier, ySupplier, zSupplier; 
    private boolean fieldOriented; 

      //constructor del chassis
    public SwerveDriveJoystickCmd(SwerveChassis swervecChassis, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, boolean fieldOriented) {
   
    this.swerveChassis = swervecChassis; 
    this.xSupplier = xSupplier; 
    this.ySupplier = ySupplier; 
    this.zSupplier = zSupplier; 
    this.fieldOriented = fieldOriented; 
    addRequirements(swervecChassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveChassis.resetHeadingHoldAfterGyroReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   
   
    /* * * ALTERING VALUES * *   */
    //Joystick values -> double 
    double xSpeed = -xSupplier.getAsDouble() * SwerveConstants.MAX_SPEED; 
    double ySpeed = -ySupplier.getAsDouble() * SwerveConstants.MAX_SPEED; 
    double zSpeed = -zSupplier.getAsDouble()*SwerveConstants.MAX_ROTATION*0.4;

    //apply deadzone to speed values 
    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 
    zSpeed = deadzone(zSpeed); 

    //square the speed values to make for smoother acceleration 
    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 
    zSpeed = modifyAxis(zSpeed); 

    swerveChassis.driveWithHeadingHold(xSpeed, ySpeed, zSpeed, fieldOriented);//FIXME SI NO FUNCIONA EL HOLDING REGRESAR A METODO DRIVE

    /*    
    if (fieldOriented) {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveChassis.getRotation2d())
      );
    } else {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
      );
    }

    swerveChassis.setModuleStates(states);
  */
} 
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveChassis.stopModules();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

/* * * ADDED METHODS * * */
public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
}

private static double modifyAxis(double num) {
  // Square the axis
  num = Math.copySign(num * num, num);

  return num;
}


}
