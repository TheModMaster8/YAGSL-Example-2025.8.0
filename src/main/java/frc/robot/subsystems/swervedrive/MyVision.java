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
/** 
public class LimeLightVision
{
    SwerveSubsystem swerveSubsystem;
    public static double rotationWhileLost = 0;
    public static double rotationAtLoss = 0;
    public static int targetID = -1; // ID of the target currently being tracked. -1 means no target.



    //get all targets in sight

    //determine which target is best either by closest or manual selection

    //get the offset from the target

    //determine the rotation and translation needed to reach the target

    //activate the commmand to seek target.

    //remember the last known target and continue to track it if it goes out of sight, ie if rotated too much left, start new recording of the rotational value since lost sight of target (lost sight > set r=0, > rot more > r= 50 CW )

    //if target is lost, rotate back to last known position and search for target again after stick input is no longer held in same direction as first lost sight or if robot is hit and rotated off course.

    //apply course correction over time to reach the target taking into account the user's input

    //stop when close enough to target or some other condition is met, cut off humam input with the exception of an over-ride key that returns control to the human. 

    //after score or task is complete, return to normal operation and end vision.




    public void StoreRotationAtLoss(double currentRotation) // In Degrees NOT radians.
    {
        rotationAtLoss = swerveSubsystem.getHeading().getDegrees();
    }

    public void UpdateRotationWhileLost(double currentRotation) // In Degrees NOT radians.
    {
        rotationWhileLost = swerveSubsystem.getHeading().getDegrees();
    }

    public double CorrectRotation() // In Degrees NOT radians.
    {
        return rotationWhileLost - rotationAtLoss;
    }

    public void StoreCurrentTarget()
    {

    }



    public void IsTargetRegained()
    {
        if()
    }

    public void GetTargetData()
    {
    // Get raw AprilTag/Fiducial data

        LimelightResults results = LimelightHelpers.getTargetCount("");

    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        for (RawFiducial fiducial : fiducials)
        {
            int id = fiducial.id;                    // Tag ID
            double txnc = fiducial.txnc;             // X offset (no crosshair)
            double tync = fiducial.tync;             // Y offset (no crosshair)
            double ta = fiducial.ta;                 // Target area
            double distToCamera = fiducial.distToCamera;  // Distance to camera
            double distToRobot = fiducial.distToRobot;    // Distance to robot
            double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
        }
    }
}
    

//NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
*/