// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import static frc.robot.Constants.Sensors.Distance.*;
import static frc.robot.commands.DriveCommands.joystickDriveRelativeCustom;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbAlign extends Command {
  Drive s_drive;
  private final double xFFKs = 0.1; // Changed from 0.2 to 0.1
  private double omegaFFKs = 0.01;
  private final double yFFKs = 0.2;
  private final double xDeadspace = 0.015; // in m
  private final double omegaDeadspace = 2; // in deg
  private final double stoppingDist = 0.1; // in m. Also known as yDeadspace
  private final double omegaPIDP = 0.1;
  private final double omegaPIDD = 0;

  PIDController yPID = new PIDController(0.7, 0.03, 0.2);
  PIDController xPID = new PIDController(1.5, 0.02, 0.03);
  PIDController omegaPID = new PIDController(0.1, 0, 0);
  SimpleMotorFeedforward xFF = new SimpleMotorFeedforward(xFFKs, 0, 0);
  SimpleMotorFeedforward omegaFF = new SimpleMotorFeedforward(omegaFFKs, 0, 0);
  SimpleMotorFeedforward yFF = new SimpleMotorFeedforward(yFFKs, 0, 0);
  private final String loggingPrefix = "commands/climb/";
  /** Creates a new Climb. */
  // Climbs the right side of the climb structure(from the perspective of the alliance station)
  public ClimbAlign(Drive s_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(s_drive);
    this.s_drive = s_drive;
  }

  // Called when the command is initially scheduled.
  // private boolean doneAligningToStart = false;
  int time = 0;
  boolean isFirstTime = false;
  long startOfTransition = 0;
  ClimbParams climbParams;
  boolean xEnabled, yEnabled, omegaEnabled;

  @Override
  public void initialize() {
    time = 0;
    setIsFirstCall(true);
    isDone = false;
    xPID.reset();
    yPID.reset();
    omegaPID.reset();
    isFirstTime = true;
    xFF.setKs(xFFKs);
    omegaFF.setKs(omegaFFKs);
    Pose2d currPose = s_drive.getPose();
    climbParams = new ClimbParams(currPose);
    Logger.recordOutput(loggingPrefix + "phase", "start");
    xEnabled = true;
    yEnabled = true;
    omegaEnabled = true;
    // xPID.setP(SmartDashboard.getNumber("xPID_P", 0.1));
    // xPID.setI(SmartDashboard.getNumber("xPID_I", 0));
    // xPID.setD(SmartDashboard.getNumber("xPID_D", 0));
    // xFF.setKs(SmartDashboard.getNumber("xFF_S", 0));
    // yPID.setP(SmartDashboard.getNumber("yPID_P", 0.1));
    // yPID.setI(SmartDashboard.getNumber("yPID_I", 0));
    // yPID.setD(SmartDashboard.getNumber("yPID_D", 0));
    // yFF.setKs(SmartDashboard.getNumber("yFF_S", 0));
    // omegaPID.setP(SmartDashboard.getNumber("omegaPID_P", 0));
    // omegaPID.setI(SmartDashboard.getNumber("omegaPID_I", 0));
    // omegaPID.setD(SmartDashboard.getNumber("omegaPID_D", 0));
    // omegaFFKs = SmartDashboard.getNumber("omegaFF_S", 0);
    // xEnabled = SmartDashboard.getBoolean("xEnabled", false);
    // yEnabled = SmartDashboard.getBoolean("yEnabled", false);
    // omegaEnabled = SmartDashboard.getBoolean("omegaEnabled", false);
    // Logger.recordOutput(loggingPrefix + "pidConsts/x/P", xPID.getP());
    // Logger.recordOutput(loggingPrefix + "pidConsts/x/I", xPID.getI());
    // Logger.recordOutput(loggingPrefix + "pidConsts/x/D", xPID.getD());
    // Logger.recordOutput(loggingPrefix + "pidConsts/x/ffS", xFF.getKs());
    // Logger.recordOutput(loggingPrefix + "pidConsts/y/P", yPID.getP());
    // Logger.recordOutput(loggingPrefix + "pidConsts/y/I", yPID.getI());
    // Logger.recordOutput(loggingPrefix + "pidConsts/y/D", yPID.getD());
    // Logger.recordOutput(loggingPrefix + "pidConsts/y/ffS", yFF.getKs());
    Logger.recordOutput(loggingPrefix + "pidConsts/omega/P", omegaPID.getP());
    Logger.recordOutput(loggingPrefix + "pidConsts/omega/I", omegaPID.getI());
    Logger.recordOutput(loggingPrefix + "pidConsts/omega/D", omegaPID.getD());
    Logger.recordOutput(loggingPrefix + "pidConsts/omega/ffS", omegaFF.getKs());
  }

  // Called every time the scheduler runs while the command is scheduled.
  private boolean isDone = false;
  // 0 = x & omega   1 = turn wheels to 90deg   2 = y
  // private boolean omegaPassed1, yPassed, xPassed = false;

  @Override
  public void execute() {
    Logger.recordOutput(loggingPrefix + "phase", "running");
    double passingX, passingY, passingOmega;
    // stoppingDist = SmartDashboard.getNumber("stoppingDist", stoppingDist);
    // boolean xEnabled = true;
    // boolean yEnabled = true;
    // boolean omegaEnabled = true;
    Logger.recordOutput(loggingPrefix + "goal", climbParams.getGoal());
    // want to go to to make sure we are going in with good alignment
    // Blue alliance

    // Note, the 2ft buffer distance is now gone
    // double directionMult = 1;
    Pose2d climbPose = climbParams.getGoal();
    Pose2d currPose = s_drive.getPose();
    Logger.recordOutput(loggingPrefix + "climbPose", climbPose);
    // X

    DoubleSupplier omegaSupplier;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;

    double xPassing = 0;
    double omegaPassing = 0;
    double yPassing = 0;
    // Start comment out
    // X
    {
      double deltaX = climbPose.getX() - currPose.getX();
      if (Math.abs(deltaX) <= 0.015) {
        xFF.setKs(0);
      } else {
        xFF.setKs(xFFKs);
      }
      xPassing = deltaX * climbParams.getXMultiplier();
    }
    // End comment out
    // Omega
    {
      // TODO FIX ANGLE WRAP
      double deltaOmega =
          climbPose.getRotation().getDegrees() - currPose.getRotation().getDegrees();
      if (Math.abs(deltaOmega) <= 2) {
        omegaFF.setKs(0);
        omegaPID.setP(0);
        omegaPID.setD(0);
      } else {
        omegaFF.setKs(omegaFFKs);
        omegaPID.setP(omegaPIDP);
        omegaPID.setD(omegaPIDD);
      }
      omegaPassing = deltaOmega * climbParams.getOmegaMultiplier();
    }
    // Y
    // Start comment out
    {
      double deltaY = climbPose.getY() - currPose.getY();
      if (Math.abs(deltaY) <= stoppingDist) {
        yFF.setKs(0);
        deltaY = 0;
        isDone = true;
      } else {
        yFF.setKs(yFFKs);
      }

      yPassing = deltaY * climbParams.getYMultiplier();
    }
    // End comment out
    Logger.recordOutput(loggingPrefix + "yDone", false);

    double pidVoltsOmega = omegaPID.calculate(omegaPassing, 0) * -1;
    double ffVoltsOmega = omegaFF.calculate(omegaPassing, 0);
    Logger.recordOutput(loggingPrefix + "controllers/omega/pidVolts", pidVoltsOmega);
    Logger.recordOutput(loggingPrefix + "controllers/omega/ffVolts", ffVoltsOmega);
    omegaSupplier = () -> pidVoltsOmega + ffVoltsOmega;

    // Start comment out
    double pidVoltsX = xPID.calculate(xPassing, 0);
    double ffVoltsX = xFF.calculate(xPassing, 0) * -1;
    Logger.recordOutput(loggingPrefix + "controllers/x/pidVolts", pidVoltsX);
    Logger.recordOutput(loggingPrefix + "controllers/x/ffVolts", ffVoltsX);
    xSupplier = () -> pidVoltsX + ffVoltsX;
    double pidVoltsY = yPID.calculate(yPassing, 0);
    double ffVoltsY = yFF.calculate(yPassing, 0) * -1;
    Logger.recordOutput(loggingPrefix + "controllers/y/pidVolts", 0);
    Logger.recordOutput(loggingPrefix + "controllers/y/ffVolts", 0);
    ySupplier = () -> MathUtil.clamp(pidVoltsY + ffVoltsY, -0.5, 0.5);
    // End comment out

    if (!yEnabled) {
      ySupplier = () -> 0;
    }
    if (!xEnabled) {
      xSupplier = () -> 0;
    }
    if (!omegaEnabled) {
      omegaSupplier = () -> 0;
    }

    Logger.recordOutput(loggingPrefix + "passing/xPassing", xPassing);
    Logger.recordOutput(loggingPrefix + "passing/omegaPassing", omegaPassing);
    Logger.recordOutput(loggingPrefix + "passing/yPassing", yPassing);

    // ySupplier = () -> 0;
    // xSupplier = () -> 0;
    // omegaSupplier = () -> 0;
    // omegaPassed = omegaPassed && time < 100; // bad practice, but its fine :)
    Logger.recordOutput(loggingPrefix + "xSupplier", xSupplier.getAsDouble());
    joystickDriveRelativeCustom(s_drive, xSupplier, ySupplier, omegaSupplier, true);
    // version of line
  }
  // set ySupplier and omegaSupplier as 0

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput(loggingPrefix + "phase", "end1");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
