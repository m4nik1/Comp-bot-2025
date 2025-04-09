// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.crypto.dsig.Transform;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class GenerateReefPoses {

    // These are in Inches!!
    double FieldLength = 690.876;
    double FieldWidth = 317;
    double ReefWidth = 65.5;
    double coralBranchSpacing = 13;
    double robotCoralIntake = 23; // This is the offset of the coral intake length wise

    double radius_reef = (ReefWidth/2) + robotCoralIntake;

    double reefX;
    double reefY = 158.5;

    Pose2d[] coralLeftPositions = new Pose2d[6];
    Pose2d[] coralRightPositions = new Pose2d[6];
    Pose2d[] algaePositions = new Pose2d[6];
    
    public GenerateReefPoses() {
        var alliance = DriverStation.getAlliance();

        if(alliance.get() == DriverStation.Alliance.Red) {
            reefX = 690 - (144 + (65.2/2));
        }
        else {
            reefX = 144 + (65.5/2);
        }

        // Generates all the positions at startup
        for(int i = 0; i < 6; i++) {
            double thetaConversion = faceToTheta(i);
            coralLeftPositions[i] = calculateCoralLeft(thetaConversion+Math.PI);
            coralRightPositions[i] = calculateCoralRight(thetaConversion+Math.PI);
            algaePositions[i] = calculateAlgaePose(thetaConversion+Math.PI);
        }
    }

    public Pose2d[] getAlgaePoses() {
        return algaePositions;
    }

    public Pose2d[] getCoralLeftPositions() {
        return coralLeftPositions;
    }

    public Pose2d[] getCoralRightPositions() {
        return coralRightPositions;
    }

    // Returns an angle in radians to calculate the poses
    public double faceToTheta(double face) {
        double theta;
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            // theta = ((2*Math.PI)*(face/6) + Math.PI) % (Math.PI);
            double t2 = ((2*Math.PI)*(face/6) + Math.PI);
            double theta_3 = t2 % (2*Math.PI); 
            theta = (theta_3 + Math.PI) % (2*Math.PI); 
        }
        else {
            double t2 = ((2*Math.PI)*(face/6));
            double theta_3 = t2 % (2*Math.PI); 
            theta = (theta_3 + Math.PI) % (2*Math.PI); 
        }
        
        System.out.println(face + ". " + theta);
        return theta;
    }

    public Pose2d calculateAlgaePose(double theta) {
        double algaeX = reefX + (radius_reef * Math.cos(theta));
        double algaeY = reefY + (radius_reef * Math.sin(theta));

        return new Pose2d(Units.inchesToMeters(algaeX), Units.inchesToMeters(algaeY), Rotation2d.fromRadians(theta + Math.PI));
    }

    public Pose2d calculateCoralLeft(double theta) {
        double coralX = reefX + (radius_reef * Math.cos(theta)) + ((coralBranchSpacing/2)*Math.cos(theta-(Math.PI/2)));
        double coralY = (reefY) + ((radius_reef) * Math.sin(theta)) + ((coralBranchSpacing/2)*Math.sin(theta-(Math.PI/2)));

        double offsetX = 14 * Math.sin(theta);
        double offsetY = 14 * (-Math.cos(theta));

        return new Pose2d(Units.inchesToMeters(coralX+offsetX), Units.inchesToMeters(coralY + offsetY), Rotation2d.fromRadians(theta + Math.PI));
    }

    public Pose2d calculateCoralRight(double theta) {
        double coralX = reefX + (radius_reef * Math.cos(theta)) + ((coralBranchSpacing/2)*Math.cos(theta+Math.PI/2));
        double coralY = (reefY) + (radius_reef * Math.sin(theta)) + ((coralBranchSpacing/2)*Math.sin(theta+Math.PI/2));

        double offsetX = 7 * Math.sin(theta);
        double offsetY = 7 * (-Math.cos(theta));

        return new Pose2d(Units.inchesToMeters(coralX + offsetX), Units.inchesToMeters(coralY + offsetY), Rotation2d.fromRadians(theta + Math.PI));
    }
}