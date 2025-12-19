package frc.robot.commands; 

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem; 

public class SwerveAutonomousCMD extends Command {
    private final DriveSubsystem swerveDrive;

    public SwerveAutonomousCMD(DriveSubsystem driveTrain) {
        this.swerveDrive = driveTrain;
        this.addRequirements(this.swerveDrive);
    }

    @Override
    public void execute() {
    
        Pose2d pos = this.swerveDrive.getPose(); 
        
        SmartDashboard.putNumber("Auto/Robot X", pos.getX());
        SmartDashboard.putNumber("Auto/Robot Y", pos.getY());
        SmartDashboard.putNumber("Auto/Robot Rotation", pos.getRotation().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        this.swerveDrive.drive(new Translation2d(0, 0), 0, false);
        //ADD FOR SAFETY!
        this.swerveDrive.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}