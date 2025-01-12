// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it u.nder the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonLoader;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Bezier;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Kinematics;
import frc.robot.subsystems.Vision;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  public static Pigeon2 pigeon = new Pigeon2(59, "canivore");

  public static Kinematics kinematics = new Kinematics(pigeon);
  public static DriveBase driveBase = new DriveBase(kinematics, pigeon);
  // public static Elevator elevator = new Elevator();
  public static Bezier bezier = new Bezier();

  public static final Vision vision = new Vision();

  public static AutonLoader autonLoader = new AutonLoader(driveBase, vision); //NEEDED SUBSYSTEMS FOR AUTON, ELEVATOR NOT USED
  public static TeleopDrive teleopDrive = new TeleopDrive(driveBase, vision); //ALL SUBSYSTEMS

  public final static CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverPort);
  public final static CommandXboxController m_manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorPort);

  // public static final PhotonSubsystem m_photonsubsystem = new PhotonSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public static void defineNamedCommands() {
    // CustomNamedCommands.registerCommand("IntakeOn", intake.IntakeDaNote(0.75).withTimeout(0.1)); 
    // CustomNamedCommands.registerCommand("IntakeOff", intake.IntakeDaNote(0.0).withTimeout(0.1)); 
    // CustomNamedCommands.registerCommand("HandoffWrist", wrist.AngleDaWrist(Constants.Wrist.handoff).withTimeout(0.1));
    // CustomNamedCommands.registerCommand("WristTest1", wrist.AngleDaWrist(Constants.Wrist.handoff).withTimeout(0.5));
    // CustomNamedCommands.registerCommand("WristTest2", wrist.AngleDaWrist(Constants.Wrist.flatWristPos).withTimeout(0.5));
    // CustomNamedCommands.registerCommand("ShootWrist", wrist.AngleDaWrist(Constants.Wrist.shoot_from_subwoofer).withTimeout(0.1));
    // CustomNamedCommands.registerCommand("ShooterOn", shooter.ShootDaNote(-0.75).withTimeout(0.1));
    // CustomNamedCommands.registerCommand("ShooterOff", shooter.ShootDaNote(0.0).withTimeout(0.1));
    // CustomNamedCommands.registerCommand("IndexOn", shooter.IndexDaNote(-0.2).withTimeout(0.1));
    // CustomNamedCommands.registerCommand("IndexReverse", shooter.IndexDaNote(0.1).withTimeout(0.1));
    // CustomNamedCommands.registerCommand("IndexOff", shooter.IndexDaNote(0.0).withTimeout(0.1));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  public static void setDriverRumble(double rumbleVal) {
    m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumbleVal);
  }

  public static Pose2d WPI_to_Custom(Pose2d pose) {
    return new Pose2d(Constants.Auton.field_size[0]-pose.getY(), pose.getX(), pose.getRotation());
  }

  public static Pose2d Custom_to_WPI(Pose2d pose) {
    return new Pose2d(pose.getY(), Constants.Auton.field_size[0]-pose.getX(), pose.getRotation());
  }

  public static Command wrappedEventCommand(Command eventCommand) {
    return new FunctionalCommand(
        eventCommand::initialize,
        eventCommand::execute,
        eventCommand::end,
        eventCommand::isFinished,
        eventCommand.getRequirements().toArray(Subsystem[]::new));
  }

    public static double getLeftJoyX() {
        if (Math.abs(m_driverController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
          return -1 * m_driverController.getLeftX();
        } else {
          return 0;
        }
      }
      public static double getLeftJoyY() {
        if (Math.abs(m_driverController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
          return m_driverController.getLeftY();
        } else {
          return 0;
        }
      }
      public static double getRightJoyX() {
        if (Math.abs(m_driverController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
          return m_driverController.getRightX();
        } else {
          return 0;
        }
      }
  // public static void setManipulatorRumble(double rumbleVal) {
  //   m_manipulatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumbleVal);
  // }

  public static double getDriverLeftJoyX() {
    if (Math.abs(m_driverController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getLeftX();
    } else {
      return 0;
    }
  }

  public static double getDriverLeftJoyY() {
    if (Math.abs(m_driverController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
      return -m_driverController.getLeftY();
    } else {
      return 0;
    }
  }

  public static double getDriverRightJoyX() {
    if (Math.abs(m_driverController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getRightX();
    } else {
      return 0;
    }
  }

  public static double getDriverRightJoyY() {
    if (Math.abs(m_driverController.getRightY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_driverController.getRightY();
    } else {
      return 0;
    }
  }

  public static Boolean getDriveBBool() {
    return m_driverController.b().getAsBoolean();
  }

  public static Boolean getDriveXBool() {
    return m_driverController.x().getAsBoolean();
  }
 

  // =========================================================
  public static double getManipulatorLeftJoyY() {
    if (Math.abs(m_manipulatorController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getLeftY();
    } else {
      return 0;
    }
  }

  public static double getManipulatorLeftJoyX() {
    if (Math.abs(m_manipulatorController.getLeftX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getLeftX();
    } else {
      return 0;
    }
  }

  public static double getManipulatorRightJoyY() {
    if (Math.abs(m_manipulatorController.getRightY()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getRightY();
    } else {
      return 0;
    }
  }

  public static double getManipulatorRightJoyX() {
    if (Math.abs(m_manipulatorController.getRightX()) > Constants.OperatorConstants.joystickDeadband) {
      return m_manipulatorController.getRightX();
    } else {
      return 0;
    }
  }

  // public static double getManipulatorRightTrigger() {
  //   if (Math.abs(m_manipulatorController.getRightTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
  //     return m_manipulatorController.getRightTriggerAxis();
  //   } else {
  //     return 0;
  //   }
  // }

  // public static double getManipulatorLeftTrigger() {
  //   if (Math.abs(m_manipulatorController.getLeftTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
  //     return m_manipulatorController.getLeftTriggerAxis();
  //   } else {
  //     return 0;
  //   }
  // }

    public static double getElevatorLeftJoystick() {
      if (Math.abs(m_manipulatorController.getLeftY()) > Constants.OperatorConstants.joystickDeadband) {
        return m_manipulatorController.getLeftY();
      } else {
        return 0;
      }
    }

    public static double getWristRightJoystick() {
      if (Math.abs(m_manipulatorController.getRightY()) > Constants.OperatorConstants.joystickDeadband) {
          return m_manipulatorController.getRightY();
        } else {
          return 0;
        }
    }

  public static double getIntakeRightTrigger() {
    if (Math.abs(m_driverController.getRightTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
          return m_driverController.getRightTriggerAxis();
        } else {
          return 0;
        }
  }

  public static double getIntakeLeftTrigger() {
    if (Math.abs(m_driverController.getLeftTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
          return m_driverController.getLeftTriggerAxis();
        } else {
          return 0;
        }
  }

  public static double getShooterRightTrigger() {
    if (Math.abs(m_manipulatorController.getRightTriggerAxis()) > Constants.OperatorConstants.joystickDeadband) {
      return -m_manipulatorController.getRightTriggerAxis();
    } else {
      return 0;
    }
  }

  public static double getShooterIntakeSpeed() {
    if(Math.abs(m_manipulatorController.getLeftTriggerAxis()) > Constants.OperatorConstants.joystickDeadband){
      return -0.35;
    } else{
      return 0.0;
    }
  }

  public static double getShooterIntakeReverseSpeed() {
    if(m_manipulatorController.leftBumper().getAsBoolean()){
      return 0.25;
    } else{
      return 0.0;
    }
  }

  public static Boolean getManipulatorBBool() {
    return m_manipulatorController.b().getAsBoolean();
  }

  public static Boolean getManipulatorABool() {
    return m_manipulatorController.a().getAsBoolean();
  }

  public static Boolean getManipulatorXBool() {
    return m_manipulatorController.x().getAsBoolean();
  }

  public static Boolean getManipulatorYBool() {
    return m_manipulatorController.y().getAsBoolean();
  }

  public static Boolean getManipulatorLeftBumperBool() {
    return m_manipulatorController.leftBumper().getAsBoolean();
  }

  public static Boolean getManipulatorRightBumperBool() {
    return m_manipulatorController.rightBumper().getAsBoolean();
  }

  public static long getFPGATime() {
    return HALUtil.getFPGATime();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonLoader.getAuton();
  }

  public void runTeleopCommand() {
    teleopDrive.schedule();
  }

  public static double findClockTime(double seconds) {
    double clocktime = (seconds/0.02);
    return clocktime;
  }

  public static ChassisSpeeds getSaturatedSpeeds(double xVel, double yVel, double omega) {
    return new ChassisSpeeds(xVel*Constants.Swerve.XPercentage, yVel*Constants.Swerve.YPercentage, omega*Constants.Swerve.angularPercentage);
  }
}
