// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

// pose's x < half field
// then climb goal's y is set to blue alliance goal's y

// poses's y < middle climb structure
// then climb goal's x is set to right alliance goal's x

// Do the other respective ones too ex. >
/** Add your docs here. */
public class ClimbParams {
  private int xMultiplier = 1; // Sign for x-direction
  private int yMultiplier = -1; // Sign for y-direction
  private int omegaMultiplier = -1; // Sign for rotation
  private boolean isFront = false;
  private Pose2d goal;

  // Notes: Left and right mean left and right if you were to face the climb structure standing on
  // the field of whatever structure you are climbing

  // TODO: Get actual x, y of these 4 cacenarios

  public ClimbParams(Pose2d estPose) {

    double x_pos = estPose.getX();
    double y_pos = estPose.getY();
    double mid_field_x = aprilTagLayout.getFieldLength() / 2;
    final String loggingPrefix = "commands/climb/climbParams/";

    if (x_pos <= mid_field_x) { // IF BLUE
      if (y_pos >= 3.75285) { // IF right (if standing in middle of field)
        // ---- NOT A TARGET CLIMB ---- //
        Logger.recordOutput(loggingPrefix + "condition", 2);
        // NOT SET YET
        xMultiplier = 1; // Sign for x-direction
        yMultiplier = -1; // Sign for y-direction
        omegaMultiplier = -1; // Sign for rotation
        goal = new Pose2d(Inches.of(41), Inches.of(177), new Rotation2d());

      } else // ELSE left
      {
        // --- THIS IS THE BLUE TARGET CLIMB ----- //
        Logger.recordOutput(loggingPrefix + "condition", 4);
        xMultiplier = -1; // VERIFIED
        yMultiplier = -1; // VERIFIED
        omegaMultiplier = 1; // VERIFIED
        goal = new Pose2d(Inches.of(41), Inches.of(120), new Rotation2d());
      }
    } // END IF BLUE
    else // ELSE red
    {
      if (y_pos <= 4.318) // IF right (If standing in center of field)
      {
        // ------ NOT A TARGET CLIMB ------ //
        // NOT SET YET
        xMultiplier = 1; // Sign for x-direction
        yMultiplier = -1; // Sign for y-direction
        omegaMultiplier = -1; // Sign for rotation
        Logger.recordOutput(loggingPrefix + "condition", 1); // Left red climb align
        goal = new Pose2d(Inches.of(250), Inches.of(136), new Rotation2d());
      } else // ELSE left
      {
        // --- THIS IS THE RED TARGET CLIMB --- //
        // NOT SET YET
        xMultiplier = -1; // NOT VERIFED
        yMultiplier = -1; // NOT VERIFIED
        omegaMultiplier = -1; // NOT VERIFIED
        Logger.recordOutput(loggingPrefix + "condition", 3);
        goal = new Pose2d(Inches.of(250), Inches.of(195), new Rotation2d());
      }
    } // END ELSE RED
  }

  public Pose2d getGoal() {
    return goal;
  }
  // These are always 1 or -1
  public int getXMultiplier() {
    return xMultiplier;
  }

  public int getYMultiplier() {
    return yMultiplier;
  }

  public int getOmegaMultiplier() {
    return omegaMultiplier;
  }
}
