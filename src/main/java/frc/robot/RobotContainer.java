package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.LoggedPowerDistribution;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawDetection;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.enums.RobotMode;

import edu.wpi.first.wpilibj.PowerDistribution;
// import frc.robot.commands.AlignToShoot;
import frc.robot.constants.Configs;
import frc.robot.constants.Constants.LimelightConstants;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */


public class RobotContainer {
  // The robot's subsystems
    // private static final RobotMode JAVA_SIM_MODE = RobotMode.SIM;
    public final LoggedPowerDistribution powerDistribution;

    // The driver's controller
    public final Controller controller = new Controller();
    
    // Initialize Sendable Chooser
//     private final SendableChooser<Command> m_chooser;
    private final SwerveDriveSimulation driveSimulation;

    private final Field2d field = new Field2d();
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      boolean isCompetition = false;
      DogLog.forceNt.log("ExampleLog/TuMadre", 0.676767);
      DogLog.log("ExampleLog/testnoforceNt", 6.7);
      DogLog.forceNt.log("ExampleLog/StringTest", "hello world");
//       m_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
//         (stream) -> isCompetition
//           ? stream.filter(auto -> auto.getName().startsWith("comp"))
//           : stream
//       );
      

      switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                driveSimulation = null;

                powerDistribution = LoggedPowerDistribution.getInstance(1, PowerDistribution.ModuleType.kRev);

                /* CTRE Chassis: */

            }

            case SIM -> {
                SimulatedArena.overrideSimulationTimings(
                        Seconds.of(0.02), Configs.DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD);
                this.driveSimulation = new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default()
                                .withRobotMass(Configs.DriveTrainConstants.ROBOT_MASS)
                                .withBumperSize(Configs.DriveTrainConstants.BUMPER_LENGTH, Configs.DriveTrainConstants.BUMPER_WIDTH)
                                .withTrackLengthTrackWidth(
                                        Configs.DriveTrainConstants.TRACK_LENGTH, Configs.DriveTrainConstants.TRACK_WIDTH)
                                .withSwerveModule(new SwerveModuleSimulationConfig(
                                        Configs.DriveTrainConstants.DRIVE_MOTOR_MODEL,
                                        Configs.DriveTrainConstants.STEER_MOTOR_MODEL,
                                        Configs.DriveTrainConstants.DRIVE_GEAR_RATIO,
                                        Configs.DriveTrainConstants.STEER_GEAR_RATIO,
                                        Configs.DriveTrainConstants.DRIVE_FRICTION_VOLTAGE,
                                        Configs.DriveTrainConstants.STEER_FRICTION_VOLTAGE,
                                        Configs.DriveTrainConstants.WHEEL_RADIUS,
                                        Configs.DriveTrainConstants.STEER_INERTIA,
                                        Configs.DriveTrainConstants.WHEEL_COEFFICIENT_OF_FRICTION))
                                .withGyro(Configs.DriveTrainConstants.gyroSimulationFactory),
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                powerDistribution = LoggedPowerDistribution.getInstance();
                // Sim robot, instantiate physics sim IO implementations
                SimulatedArena.getInstance().resetFieldForAuto();
            }

            default -> {
              driveSimulation = null;
              powerDistribution = LoggedPowerDistribution.getInstance();
            }
        }

        // SmartDashboard.putData("Auto Chooser", m_chooser);
        // m_chooser.addOption("pleasework", new ThruTakeToShooter(ThroughTake.getInstance(), Shooter.getInstance()));
        // m_chooser.addOption("Test PP_LL AutoAlign", DriveTrain.getInstance().ppLLTestAlign());

        SmartDashboard.putData("Field", field);

        NamedCommands.registerCommand("IntakeOn", Intake.getInstance().runOnce(() -> Intake.getInstance().moveIntakeDown()));
        NamedCommands.registerCommand("StopIntake", Intake.getInstance().runOnce(() -> Intake.getInstance().moveIntakeUp()));

        NamedCommands.registerCommand("ThruTake", ThroughTake.getInstance().runOnce(() -> ThroughTake.getInstance().runThroughTake(1)));
        NamedCommands.registerCommand("StopThruTake", ThroughTake.getInstance().runOnce(() -> ThroughTake.getInstance().stopThroughTake()));
        
        NamedCommands.registerCommand("Shoot", Shooter.getInstance().runOnce(() -> Shooter.getInstance().setShooterSpeed(0.7)));
        NamedCommands.registerCommand("StopShooter", Shooter.getInstance().runOnce(() -> Shooter.getInstance().setShooterSpeed(0)));

        // Pathplanner Registered Event Markers
        // new EventTrigger("test_pose_align_shoot").whileTrue(AlignToShoot.getInstance());
        // NamedCommands.registerCommand("test_pose_align_shoot", AlignToShoot.getInstance());

        // m_chooser.addOption("Fuel Object Detection Align", DriveTrain.getInstance().pathplannerObjAlign());

        // m_chooser.addOption("Testing shooter align", new AlignToShoot(DriveTrain.getInstance()));

      // Ignore controller warnings
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void resetSimulationField() {
      if (!Robot.isSimulation()) return;

      DriveTrain.getInstance().resetOdometry(new Pose2d(3, 3, new Rotation2d()));
      SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
      if (!Robot.isSimulation()) return;

      SimulatedArena.getInstance().simulationPeriodic();
      org.littletonrobotics.junction.Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
      Logger.recordOutput(
              "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
      Logger.recordOutput(
              "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  // public LimelightHelpers.RawDetection[] llDetectTest() {
  //   RawDetection[] m_detect = LimelightHelpers.getRawDetections(LimelightConstants.limelight_three);
  //   for (RawDetection detection : m_detect) {
  //     System.out.println("Class: " + detection.classId);
  //     System.out.println("TA" + detection.ta);
  //     System.out.println("TX" + detection.txnc);
  //     System.out.println("TY" + detection.tync);
  //   }
  //   return m_detect;
  //   // System.out.println("Neural Class ID" + LimelightHelpers.getNeuralClassID(LimelightConstants.limelight_three));
  //   }
    // DogLog.forceNt.log("Limelight/raw_detect", LimelightHelpers.getDetectorClass(LimelightConstants.limelight_three));
    // System.out.println(m_detect.getClass());

    // System.out.println(LimelightHelpers.getDetectorClass(LimelightConstants.limelight_three));
    
  // }
  /* 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        // return new InstantCommand(
        //         () -> Shooter.getInstance().setShooterSpeed(0.6),
        //         Shooter.getInstance()
        // );

        return new ThruTakeToShooter(ThroughTake.getInstance(), Shooter.getInstance());

        // return new InstantCommand(
        //         () -> new ThruTakeToShooter(ThroughTake.getInstance(), Shooter.getInstance())
        // );
//     return m_chooser.getSelected();
  }
}