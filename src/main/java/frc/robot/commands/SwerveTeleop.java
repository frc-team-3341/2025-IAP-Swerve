package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.util.lib.ArcadeJoystickUtil;
import frc.util.lib.AsymmetricLimiter;

public class SwerveTeleop extends Command {
    private DriveSubsystem swerveDriveTrain;
    private CommandXboxController joystick;
    private double robotSpeed = 2.0;

    private double xMult = 1.0;
    private double yMult = 1.0;
   
    // Slew rate limit controls
   // Positive limit ensures smooth acceleration (1000 * dt * dControl)
   // Negative limit ensures an ability to stop (0 * dt * dControl)
   private final AsymmetricLimiter translationLimiter = new AsymmetricLimiter(5.0D, 1000.0D);
   private final AsymmetricLimiter rotationLimiter    = new AsymmetricLimiter(10.0D, 10.0D);
    /**
   * Creates a SwerveTeleop command, for controlling a Swerve bot.
   *
   * @param swerve          - the Swerve subsystem
   * @param joy             - joystick controller
   */
    public SwerveTeleop(DriveSubsystem swerve, CommandXboxController joy){
        this.swerveDriveTrain = swerve;
        this.joystick = joy;
        this.addRequirements(swerveDriveTrain);
    }
    @Override
    public void execute(){
        //Flip all controller input since xbox forward is negative
      double xVal = -this.joystick.getRawAxis(XboxController.Axis.kLeftY.value);
      double yVal = -this.joystick.getRawAxis(XboxController.Axis.kLeftX.value);
      double rotation = -this.joystick.getRawAxis(XboxController.Axis.kRightX.value);
      double translationRightTrigger = this.joystick.getRawAxis(XboxController.Axis.kRightTrigger.value);
      //boolean resetNavx = this.joystick.getRawButtonPressed(XboxController.Button.kY.value);

      //Toggles the field relative state
      //if (this.joystick.getRawButtonPressed(XboxController.Button.kX.value)){this.fieldRelative = !this.fieldRelative;}; 

      double rightTriggerVal = Math.abs(translationRightTrigger);

      
      if (frc.robot.Constants.SwerveConstants.fastMode) {
         //Will be 100% speed by default or decrease speed by how far pressed right trigger is
         //Minimum speed is 10%
         rightTriggerVal = Math.max((1.0 - rightTriggerVal), .1);
      } else {
         //Will be 10% speed by default or increase speed by how far pressed right trigger is
         //Minimum speed is 10%
         rightTriggerVal = Math.max(rightTriggerVal, 0.1);
      }
        xVal = MathUtil.applyDeadband(xVal, Constants.SwerveConstants.deadBand);
        yVal = MathUtil.applyDeadband(yVal, Constants.SwerveConstants.deadBand);
        rotation = MathUtil.applyDeadband(rotation, Constants.SwerveConstants.deadBand);
        
        double rotationVal = MathUtil.applyDeadband(rotation, Constants.SwerveConstants.deadBand);

        // Apply rate limiting to rotation
        rotationVal = this.rotationLimiter.calculate(rotationVal);
       
        double[] polarCoor = ArcadeJoystickUtil.regularGamePadControls(xVal, yVal, 1.5);
        
        double newHypot = robotSpeed*translationLimiter.calculate(polarCoor[0]);

        // Deadband should be applied after calculation of polar coordinates
        newHypot = MathUtil.applyDeadband(newHypot, Constants.SwerveConstants.deadBand);
  
        //Through magic we can convert the controller input into a vector that can be applied to the robot
        double correctedX = rightTriggerVal * xMult * newHypot * Math.cos(polarCoor[1]);
        double correctedY =  rightTriggerVal * yMult * newHypot * Math.sin(polarCoor[1]);

         this.swerveDriveTrain.drive(new Translation2d(correctedX, correctedY),
            rotationVal * Constants.SwerveConstants.maxChassisAngularVelocity,
             false);

    }

    @Override
    public void end(boolean interrupted){
        this.swerveDriveTrain.drive(new Translation2d(0, 0), 0, false);
        this.swerveDriveTrain.stopMotors();
    }
}
