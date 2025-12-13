// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * WARNING: PLEASE USE UTF-8 and not WINDOWS-1252 character encoding
 * The element symbol (weird e character) is UTF-8 only
 * Thank you future reader :)
 * 
 * This only occured due to the 2024 update
 * 
 * FIX:
 * ADD THIS TO YOUR BUILD.GRADLE (WITHOUT ASTERISKS)
 * compileJava.options.encoding = 'UTF-8' 
 * compileTestJava.options.encoding = 'UTF-8'
 * javadoc.options.encoding = 'UTF-8'
 * 
 * Your error will look like this if you use Window's encoding:
 * ArcadeJoystickUtil.java:47: error: unmappable character (0x9D) for encoding windows-1252
 */

/** A class providing a helper method for converting x-y control (x e [-1, 1], y e [-1, 1]) to scaled polar coordinates (arcade drive/stick). See convertXYToScaledPolar for more details */
public class ArcadeJoystickUtil {

    /**
     * The empty default constructor of this utility class.
     */
    public ArcadeJoystickUtil() {}

    // Regular signed angle
    double controlsAngle = 0.0;

    // Hypotenuse from controls
    double controlsHypot = 0.0;

    public static double[] regularGamePadControls(double x, double y, double maxMagnitude){
        double[] controlVar = new double[2];
        controlVar[0] = Math.hypot(x, y)*maxMagnitude;
        controlVar[1] =  Math.atan2(y, x);

        return controlVar;
    }
    
}
