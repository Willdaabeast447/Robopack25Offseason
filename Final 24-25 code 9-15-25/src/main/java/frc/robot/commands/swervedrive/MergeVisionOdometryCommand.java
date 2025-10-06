package frc.robot.commands.swervedrive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

/**
 * MergeVisionOdometryCommand continuously reads vision data from both
 * limelights,
 * checks if each measurement's confidence is above a defined threshold, and
 * updates the drivetrain's odometry.
 *
 * IMPORTANT: This command does not require the drivetrain subsystem, so it can
 * run concurrently with your main swerve drive command.
 * It is assumed that your drivetrain (of type CommandSwerveDrivetrain)
 * has a method called addVisionMeasurement(Pose2d visionPose, double timestamp)
 * to fuse the vision data with its odometry.
 */
public class MergeVisionOdometryCommand extends Command {
    // Reference to the Vision subsystem (provides access to the two limelights)
    private final Vision vision;
    // Reference to the drivetrain used sdolely for updating odometry
    private final SwerveSubsystem drivetrain;
    // Define a minimum confidence threshold (you can adjust this value as needed)
    private final double MIN_CONFIDENCE = 0.5;

    /**
     * Constructor for the MergeVisionOdometryCommand.
     * 
     * @param vision     The Vision subsystem containing the limelights.
     * @param drivetrain The drivetrain that will have its odometry updated.
     */
    public MergeVisionOdometryCommand(Vision vision, SwerveSubsystem drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        // Only add vision as a requirement here so that it doesn't block the drivetrain
        // command.
        addRequirements(vision);
        // Do NOT add the drivetrain as a requirement so both commands can run
        // concurrently.
    }

    @Override
    public void execute() {
        // Retrieve the vision pose estimates from both limelights.
        LimelightHelpers.PoseEstimate bowEstimate = vision.bowLL.getVisionPose();
        LimelightHelpers.PoseEstimate aftEstimate = vision.aftLL.getVisionPose();

       /*  // Check if the bowl (front) limelight sees a target and if its confidence meets
        // the threshold.
        if (vision.bowLL.getTV() && vision.bowLL.getConfidence() >= MIN_CONFIDENCE) {
            // If so, update the drivetrain odometry with the bowl estimate.
            drivetrain.addVisionMeasurement(
                    bowEstimate.pose,
                    Utils.fpgaToCurrentTime(
                            bowEstimate.timestampSeconds),
                    VecBuilder.fill(0.1, 0.1, 0100));
        }

        // Check if the aft (rear) limelight sees a target and if its confidence meets
        // the threshold.
        if (vision.aftLL.getTV() && vision.aftLL.getConfidence() >= MIN_CONFIDENCE) {
            // If so, update the drivetrain odometry with the aft estimate.
            drivetrain.addVisionMeasurement(
                    aftEstimate.pose,
                    Utils.fpgaToCurrentTime(aftEstimate.timestampSeconds),
                    VecBuilder.fill(0.1, 0.1, 0100));
        } */
    }

    @Override
    public boolean isFinished() {
        // This command is designed to run continuously, updating odometry as new vision
        // data comes in.
        return false;
    }
}
