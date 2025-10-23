package frc.robot.subsystems.swervedrive;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;



public class MyVision
{
    
    // Data class to hold all the April tag's information.
    public class AprilData
    {
        public double id;
        public String family;
        public Pose3d robotPoseField;
        public Pose3d cameraPoseTag;
        public Pose3d robotPoseTag;
        public Pose3d tagPoseCamera;
        public Pose3d tagPoseRobot;
        public double tx;
        public double ty;
        public double ta;
        
        // Constructor
        public AprilData(double id, String family, Pose3d robotPoseField, Pose3d cameraPoseTag, Pose3d robotPoseTag, Pose3d tagPoseCamera, Pose3d tagPoseRobot, double tx, double ty, double ta)
        {
            this.id = id;
            this.family = family;
            this.robotPoseField = robotPoseField;
            this.cameraPoseTag = cameraPoseTag;
            this.robotPoseTag = robotPoseTag;
            this.tagPoseCamera = tagPoseCamera;
            this.tagPoseRobot = tagPoseRobot;
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
        }
    }

    public static void ParseRawResults()
    {
        LimelightResults results = LimelightHelpers.getLatestResults(""); // Get the latest results from the Limelight.
        if (results.valid) 
        {
            if (results.targets_Fiducials.length > 0) // If we detected any fiducials otherwise don't waste time and CPU.
            {
                LimelightTarget_Fiducial tag = results.targets_Fiducials[0]; // Get the first detected fiducial (probably the right most one).
                double id = tag.fiducialID;          // Tag ID
                String family = tag.fiducialFamily;   // Tag family (e.g., "16h5")
                // 3D Pose Data
                Pose3d robotPoseField = tag.getRobotPose_FieldSpace();    // Robot's pose in field space
                Pose3d cameraPoseTag = tag.getCameraPose_TargetSpace();   // Camera's pose relative to tag
                Pose3d robotPoseTag = tag.getRobotPose_TargetSpace();     // Robot's pose relative to tag
                Pose3d tagPoseCamera = tag.getTargetPose_CameraSpace();   // Tag's pose relative to camera
                Pose3d tagPoseRobot = tag.getTargetPose_RobotSpace();     // Tag's pose relative to robot
                // 2D targeting data
                double tx = tag.tx;                  // Horizontal offset from crosshair
                double ty = tag.ty;                  // Vertical offset from crosshair
                double ta = tag.ta;                  // Target area (0-100% of image)

                System.out.println("ID: " + id + ", tx: " + tx + ", ty: " + ty + ", ta: " + ta);

            }
        //return new AprilData(id, family, robotPoseField, cameraPoseTag, robotPoseTag, tagPoseCamera, tagPoseRobot, tx, ty, ta);
        }
    }
}
