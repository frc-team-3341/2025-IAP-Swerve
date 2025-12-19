// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
// PathPlanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig; 
import com.pathplanner.lib.config.PIDConstants; 
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// WPILib Imports for Auto
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.util.lib.SwerveUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      SwerveConstants.kFrontLeftDrivingCanId,
      SwerveConstants.kFrontLeftTurningCanId, 9,
      SwerveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      SwerveConstants.kFrontRightDrivingCanId,
      SwerveConstants.kFrontRightTurningCanId, 10, 
      SwerveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      SwerveConstants.kRearLeftDrivingCanId,
      SwerveConstants.kRearLeftTurningCanId, 12, 
      SwerveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      SwerveConstants.kRearRightDrivingCanId,
      SwerveConstants.kRearRightTurningCanId, 11,
      SwerveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  
private AHRS navx = new AHRS(NavXComType.kMXP_SPI);
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      SwerveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(navx.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

private Rotation2d offsetNavx = Rotation2d.fromDegrees(0);
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    this.kinematics = Constants.SwerveConstants.kDriveKinematics;
   this.moduleIO = new MAXSwerveModule[] {
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight
    };
this.poseEstimator = new SwerveDrivePoseEstimator(
  SwerveConstants.kDriveKinematics,
  navx.getRotation2d(),
  new SwerveModulePosition[] {
      m_frontLeft.getPosition(), m_frontRight.getPosition(),
      m_rearLeft.getPosition(), m_rearRight.getPosition()
  },
  new Pose2d()
);
    
      // Usage reporting for MAXSwerve template
      
   
    createAuto();
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveDriveSimulation mapleSimDrive;
  public Pose2d getPoseFromEstimator() {
    return poseEstimator.getEstimatedPosition();
 }
 public void resetPose(Pose2d pose) {
  // We call the method here to get REAL data instead of using the empty variable
  SwerveModulePosition[] currentPositions = getModulePositions();
  
  poseEstimator.resetPosition(pose.getRotation(), currentPositions, pose);
  offsetNavx = pose.getRotation().minus(navx.getRotation2d());

  if (Constants.SwerveConstants.isSim && mapleSimDrive != null) {
      mapleSimDrive.setSimulationWorldPose(pose);
  }
}

public void driveRelative(ChassisSpeeds speeds) {
  speeds = SwerveUtil.discretize(speeds, -4.0);

  SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(speeds);

  // MUST USE SECOND TYPE OF METHOD
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speeds,
        Constants.SwerveConstants.maxWheelLinearVelocityMeters,
        Constants.SwerveConstants.maxChassisTranslationalSpeed,
        Constants.SwerveConstants.maxChassisAngularVelocity);

  for (int i = 0; i < swerveModuleStates.length; i++) {
     this.moduleIO[i].setDesiredState(swerveModuleStates[i]);
  }
}
private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private void createAuto()  {
    try {
       RobotConfig config = RobotConfig.fromGUISettings();

       AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants                                                      
          ),
          config,
          () -> {
             // Boolean supplier that controls when the path will be mirrored for the red
             // alliance
             // This will flip the path being followed to the red side of the field.
             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
             var alliance = DriverStation.getAlliance();
             if (alliance.isPresent()) {
             return alliance.get() == DriverStation.Alliance.Red;
             }
             return false;
          },
          this); // Reference to this subsystem to set requirements
      autoChooser = AutoBuilder.buildAutoChooser("S2_H1_C2_Auto");
       SmartDashboard.putData(autoChooser);
    } catch (Exception e) {
       //If an exception is thrown here we are really in trouble
       e.printStackTrace();
       System.out.println("uh oh auto is really broken");
    }  
 }
public void driveRobotRelative(ChassisSpeeds speeds) {
  this.drive(
      new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), 
      speeds.omegaRadiansPerSecond, 
      false 
  );
}


public ChassisSpeeds getRobotRelativeSpeeds() {
  return SwerveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
  );
}
public SwerveModulePosition[] getModulePositions() {
  return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  };
}
public Command resetHeadingCommand() {
  return runOnce(() -> {
     navx.reset();
  });
}
@Override
public void periodic() {
    
    m_odometry.update(getRotation(), getModulePositions());
    
    
    poseEstimator.update(getRotation(), getModulePositions());
}

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {

    return poseEstimator.getEstimatedPosition();
}

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(navx.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  public void stopMotors() {
    for (MAXSwerveModule module : moduleIO) {
       module.setDriveVoltage(0.0);
       module.setTurnVoltage(0.0);
    }
 }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  private boolean fieldRelative = true;
  private SwerveDriveKinematics kinematics;
  private ChassisSpeeds chassisSpeeds;
  private MAXSwerveModule[] moduleIO;
 public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
      //This question mark and colon are called ternary operators
      //If field relative is true, then do the line with the ?, if false do :
      this.chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                  this.getRotation())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

            this.chassisSpeeds = SwerveUtil.discretize(this.chassisSpeeds, -4);

      // Convert the robot vector into module states which is a vector for each module
      // Explanation found here
      // https://samliu.dev/blog/a-deep-dive-into-swerve#16d4e0ca3f0280b19d85cdb8b2adac83
      SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

      // MUST USE SECOND TYPE OF METHOD
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, this.chassisSpeeds,
            Constants.SwerveConstants.maxWheelLinearVelocityMeters,
            Constants.SwerveConstants.maxChassisTranslationalSpeed,
            Constants.SwerveConstants.maxChassisAngularVelocity);

      for (int i = 0; i < swerveModuleStates.length; i++) {
         this.moduleIO[i].setDesiredState(swerveModuleStates[i]);
      }
   }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.maxWheelLinearVelocityMeters);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }
  public double getGyroYaw() {
    return navx.getYaw();
 }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -navx.getRotation2d().plus(offsetNavx).getDegrees();
  }
  public Rotation2d getRotation() {
    return navx.getRotation2d().plus(offsetNavx);
 }
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navx.getRate();
  }
  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
}
}