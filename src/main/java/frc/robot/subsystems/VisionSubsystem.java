package frc.robot.subsystems;

import java.util.Arrays;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class VisionSubsystem extends SubsystemBase {
    
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tid = getDefault().getTable("limelight").getEntry("tid");
    private NetworkTableEntry test = getDefault().getTable("limelight").getEntry("botpose");
    private NetworkTableEntry botpose = getDefault().getTable("limelight").getEntry("targetpose_cameraspace");
    private Alliance alliance;
    private Boolean validAprilTags = true;
    private Timer timer = new Timer();
    private double tx0 = 13.7;
    private double ty0 = 10.1;
    private double tx1 = 9.19;
    private double ty1 = 7.18;
    private NetworkTableEntry tx = getDefault().getTable("limelight").getEntry("tx");
    private NetworkTableEntry ty = getDefault().getTable("limelight").getEntry("ty");
    private NetworkTableEntry tl = getDefault().getTable("limelight").getEntry("tl");
    private NetworkTableEntry cl = getDefault().getTable("limelight").getEntry("cl");
    private NetworkTableEntry ta = getDefault().getTable("limelight").getEntry("ta");
    private NetworkTableEntry json = getDefault().getTable("limelight").getEntry("json");
    private double x;
    private double y;
    private String Alliance;
    private double fieldWidth = 8.0137;//in meters
    private double fieldLength = 16.5417;
    private double[] visionPose = new double[6];
    public double dist0XStdDev = 1.6;//0.0475;
    public double dist0YStdDev = 1.6;//0.0850;
    public double dist1XStdDev = 3.2;//0.0156;
    public double dist1YStdDev = 3.2;//0.0324;
    public double dist2XStdDev = 8;//0.0253;
    public double dist2YStdDev = 8;//0.0519;
    public double placeXStdDev = 0.6;
    public double placeYStdDev = 0.6;
    private double txBound = 2; //this is the range of tx 2 that makes the y position of the limelight goofy. We check this every time. Dont ask me why
    private double l = 0; //stored from tl;
    private double c = 0; //stored from cl
    private double a = 0;
    // private int first = -1;
    // private int last = -1;
    // private int fiducialIndex = -1;
    // String m_json = "";
    private int aprilTags = 0;
    private double [] relativeVisionPose;




    public LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
    
    public LimelightHelpers.LimelightTarget_Fiducial[] fiducials = llresults.targetingResults.targets_Fiducials;



    // double id
    // double[] pose;
    int index = 0;

    /*
     * NOTE
     * you must be ignoring the pipeline networktables index for the code
     * to be able to change the pipeline
     */
    public VisionSubsystem () {
        //timer.start();
        index = 0;
        table.getEntry("pipeline").setNumber(index);
        

        //updatePipeline();
        // table.getEntry("getpipe").setDouble((double) index);
        // getDefault().getTable("limelight").getEntry("pipeline").setDouble(2);
    }


    @Override
    public void periodic () {
        // THIS WORKS
        // id = tid.getDouble(-1);
        // if (id == -1) {
        //    //validAprilTag = false;
        //     zoomOut();
        // } else {
        //    //validAprilTag = true;
        //     canWeZoom();
        // }
        // if (validAprilTag) {

        // llresults.getAlliance();
        // LimelightHelpers.LimelightTarget_Fiducial = llresults.results.targets_Fiducials
        // LimelightHelpers.LimelightTarget_Fiducial = m_results.results.targets_Fiducials;
        LimelightHelpers.getJSONDump("limelight");
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
        //LimelightHelpers.LimelightTarget_Fiducial[] bruh = llresults.results.targets_Fiducials;
        




        // llresults = LimelightHelpers.getLatestResults("");
        fiducials = llresults.targetingResults.targets_Fiducials;


        //a = ta.getDouble(0);
        validAprilTags = (fiducials.length >= 2);
        visionPose = getVisionTags(); 
        relativeVisionPose = visionRelativeOdometry();
        SmartDashboard.putNumber("#Tags", 1);
        // SmartDashboard.putString("JSON", json.getString("{}"));
        SmartDashboard.putBoolean("Valid April Tag", validAprilTags);
        SmartDashboard.putNumber("xPose", visionPose[0]);
        SmartDashboard.putNumber("yPose", visionPose[1]);
        SmartDashboard.putNumber("tl", tl.getDouble(999));
        SmartDashboard.putNumber("cl", cl.getDouble(999));
        //SmartDashboard.putNumber("Tags#", aprilTags);

            // m_json = json.getString("{}");
            // first = m_json.indexOf("Fiducial");
            // last = m_json.indexOf("Retro");
            // fiducialIndex = m_json.indexOf("},{", first);
            // aprilTags = validAprilTag ? 1 : 0;
            // //System.out.println(fiducialIndex);
            // if (fiducialIndex != -1 && fiducialIndex < last) { // at least two
            //     aprilTags = 2;
            //     first = m_json.indexOf("},{", fiducialIndex + 1);
            //     fiducialIndex = m_json.indexOf("},{", first + 1);
            //     // System.out.println(fiducialIndex);
            //     if (fiducialIndex != -1 && fiducialIndex < last) { // at least three
            //         aprilTags = 3;
            //         first = m_json.indexOf("},{", fiducialIndex + 1);
            //         fiducialIndex = m_json.indexOf("},{", first + 1);
            //         // System.out.println(fiducialIndex);
            //         if (fiducialIndex != -1 && fiducialIndex < last) {
            //             aprilTags = 4;
            //         }
            //     }
            // }
            

            
            // JSONObject m_json = new JSONObject(table.getEntry("json").getString("{}"));
        //}
        
        
        // if (timer.advanceIfElapsed(1)) {
        //     updatePipeline();
        // }
        // id = tid.getDouble(-1);
        // if (index != 2) {
        //     updatePipeline();
        // } else {
        //     validAprilTag = true;
        //     SmartDashboard.putBoolean("Valid April Tag", validAprilTag);
        //     //optimizeVision();
        // }
    }

    /*

    public void zoomOut() {
        if (index != 0) {
            index --;
            table.getEntry("pipeline").setNumber(index);
        }
        validAprilTags = false;
    }

    public void canWeZoom () {
        validAprilTags = true;
        x = tx.getDouble(-99);
        y = ty.getDouble(-99);
        if (index != 2) {
            if (index == 1) {
                if(Math.abs(x) <= tx1 && Math.abs(y) <= ty1) {
                    index = 2; //we can zoom!
                    table.getEntry("pipeline").setNumber(index);
                    validAprilTags = false; //false because while we transition we may lose the tag for a bit
                }
            }

            if (index == 0) {
                if (Math.abs(x) <= tx0 && Math.abs(y) <= ty0) {
                    index = 1;// we can zoom!
                    table.getEntry("pipeline").setNumber(index);
                    validAprilTags = false; //false because while we transition we may lose the tag for a bit
                }
            }
        }
        SmartDashboard.putNumber("tx", x);
    }

    */
    public boolean canWePlace () {
        return numFiducials() == 1;
    }
    /**
     * Returns whether you have trustable vision on at least two AprilTags
     * @param navx
     * @return boolean of vision trustworthiness
     */
    public Boolean validVision (AHRS navx) { //navx casue we want to make sure we aren't tilted
        SmartDashboard.putBoolean("validAprilTag", (validAprilTags && (Math.abs(navx.getPitch()) <= 7)));
        return (validAprilTags) && (Math.abs(navx.getPitch()) <= 7); //x is tx
    }

    public int numFiducials () {
        return fiducials.length;
    }

    public void visionPlaceOdometry (SwerveDrivePoseEstimator estimator, Rotation2d navxRotation) {
        c = cl.getDouble(16); //16 is normally what it is
        l = tl.getDouble(60); //60 is an average number
        if (!(relativeVisionPose[0] == 0)) { //if we actually have valid
            Pose2d visionPose2d = new Pose2d(visionPose[0], visionPose[1], navxRotation);
            estimator.addVisionMeasurement(visionPose2d, Timer.getFPGATimestamp() - (c/1000) -(l/1000));
        }
    }


    public double[] visionRelativeOdometry() {
        double id = (int) getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        double[] vision;
        if (id != -1) { //TODO fix this
           vision =  getDefault().getTable("limelight").getEntry("tagpose").getDoubleArray(new double[6]);
            return vision;
            
        } else {
            return new double[6];
        }
    }
    /**
     * 
     * Update odometry of the robot using AprilTag vision
     * The farther away you are from the targets, the less you trust the data
     * Sets the latency variables (c and l)
     * @param estimator The drivetrain estimator for odometry
     * @param navxRotation Rotation from the navx (trustworthy)
     */
    public void visionOdometryUpdate(SwerveDrivePoseEstimator estimator, Rotation2d navxRotation) {
        c = cl.getDouble(16); //16 is normally what it is
        l = tl.getDouble(60); //60 is an average number
        if (estimator.getEstimatedPosition().getX() <= 3){
            estimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(dist0XStdDev, dist0YStdDev, 0.01));
        }
        if (estimator.getEstimatedPosition().getX() > 3 && estimator.getEstimatedPosition().getX() <= 5) {
            estimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(dist1XStdDev, dist1YStdDev, 0.01));
        }
        if (estimator.getEstimatedPosition().getX() > 5) {
            estimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(dist2XStdDev, dist2YStdDev, 0.01));
        }
        Pose2d visionPose2d = new Pose2d(visionPose[0], visionPose[1], navxRotation);
        estimator.addVisionMeasurement(visionPose2d, Timer.getFPGATimestamp() - (c/1000) -(l/1000));
    }

    public double[] getRelativePose() {
        double id = (int) getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        double[] vision;
        if (id != -1) {
            //double[] vision = getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
            //Alliance = DriverStation.getAlliance().toString(); //this may be inefficent
            //SmartDashboard.putString("Alliance", Alliance);
            //if (Alliance == "Blue") { //these are our coords for blue alliance
                // vision[0] = vision[0] + fieldLength/2; //distance to center of feild
                // vision[1] = vision [1] + fieldWidth/2;
            vision = getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            //} else { //red alliance coords
                //vision = getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
                // vision[0] = fieldLength/2-vision[0];
                // vision[1] = fieldWidth/2-vision[1];
                // vision[5] = -vision[5]; //i think this is jsut because we are on opposite sides of the feild
            //}
            return vision;
            
        } else {
            return new double[6];
        }
    }
    /**
     * Get the field-relative location of the robot while accounting for the alliance color (red/blue)
     * @return 3d botpose
     */
    public double[] getVisionTags () {
        double id = (int) getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        double[] vision;
        if (id != -1) {
            //double[] vision = getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
            Alliance = DriverStation.getAlliance().toString(); //this may be inefficent
            //SmartDashboard.putString("Alliance", Alliance);
            if (Alliance == "Blue") { //these are our coords for blue alliance
                // vision[0] = vision[0] + fieldLength/2; //distance to center of feild
                // vision[1] = vision [1] + fieldWidth/2;
                vision = getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            } else { //red alliance coords
                vision = getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
                // vision[0] = fieldLength/2-vision[0];
                // vision[1] = fieldWidth/2-vision[1];
                // vision[5] = -vision[5]; //i think this is jsut because we are on opposite sides of the feild
            }
            return vision;
            
        } else {
            return new double[6];
        }
        
        //System.out.println(Arrays.toString(vision));    
    }
}
