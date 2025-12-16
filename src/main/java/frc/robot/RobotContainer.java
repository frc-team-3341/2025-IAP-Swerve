package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(8.7, 4.0), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final CommandXboxController drivingXbox = new CommandXboxController(0);

  private SwerveDriveTrain swerveDriveTrain;
  private SwerveTeleopCMD swerveTeleopCMD;

  public RobotContainer() {
    Command path = AutoBuilder.buildAuto("1Auto.auto");
    createSwerve();

  }

  private void createSwerve() {
    //Create swerveDriveTrain
    swerveDriveTrain = new SwerveDriveTrain(startpose,
    Constants.SwerveModuleIOConfig.moduleFL,
    Constants.SwerveModuleIOConfig.moduleFR,
    Constants.SwerveModuleIOConfig.moduleBL,
    Constants.SwerveModuleIOConfig.moduleBR);
    
    //Create swerve commands here
    swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);


    //Set default swerve command to the basic drive command, not field orientated
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);

    //This requires the swerve subsystem make sure to create that first before creating this
    // drivingXbox.x().onTrue(this.swerveDriveTrain.toggleFieldCentric());
    drivingXbox.y().onTrue(this.swerveDriveTrain.resetHeadingCommand());

  }

  public Command getAutonomousCommand() {
    new EventTrigger("1Auto.auto");
    return null;
  }

}