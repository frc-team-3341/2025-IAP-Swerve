package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.util.lib.ArcadeJoystickUtil;

public class SwerveTeleop extends Command {
    private DriveSubsystem dt;
    private CommandXboxController joy;

    public SwerveTeleop(DriveSubsystem dt, CommandXboxController joy){
        this.dt = dt;
        this.joy = joy;

        addRequirements(dt);
    }
    //@Override
    public void execute(){
        double xVal = joy.getLeftY();
        double yVal = joy.getLeftX();
        double rotation = joy.getRightX();

        xVal = MathUtil.applyDeadband(xVal, Constants.SwerveConstants.deadBand);
        yVal = MathUtil.applyDeadband(yVal, Constants.SwerveConstants.deadBand);
        rotation = MathUtil.applyDeadband(rotation, Constants.SwerveConstants.deadBand);

        double[] polarCoor = ArcadeJoystickUtil.regularGamePadControls(xVal, yVal, 1.5);
        
        xVal = Math.cos(polarCoor[1]) * polarCoor[0];
        yVal = Math.sin(polarCoor[1]) * polarCoor[0];

        dt.drive(xVal, yVal, rotation, false);        

    }

    //@Override
    public void end(boolean interrupted){
        dt.drive(0,0,0,false);

    }
}
