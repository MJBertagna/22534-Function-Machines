import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class BotConstants {

    // "H" Stands for Horizontal in any variable names

    public static FtcDashboard dashboard;

    public static IMU imu;

    //Vertical Slide Variables
//    public static double PIDSign = 1;
    public static double integralSum = 0;
    public static double Kp = 0.013;
    public static double Ki = 0;
    public static double Kd = 0;

    //Horizontal Slide variables
    public static double integralSumH = 0;
    public static double KpH = 0.005;
    public static double KiH = 0;
    public static double KdH = 0.0002;

    //Reference variables
    public static int referenceMax = 2500;
    public static int referenceHang = 1000;

    public static int referenceHMax = 1460;
    public static int referenceHMid = 800;
    public static int referenceHShort = 300;

    public static int counter = 0;
    public static int counterH = 0;
    public static int reference = 0;//Reference for the target position of the vertical slide
    public static int referenceH = 0;//Reference for the target position of the horizontal slide

    public static int[] references = {10, referenceMax};
    public static int[] referencesH = {referenceHShort, referenceHMid, referenceHMax};

    public static boolean isRightBumperPressed = false;
    public static boolean isLeftBumperPressed = false;

    public static boolean isSquarePressed = false;
    public static boolean isTrianglePressed = false;
    public static boolean isCrossPressed = false;

    //Maybe use maybe not
    public static int timesSquarePressed = 0;

    //Claw reference variables for open and close positions
    //Correct
    public static double clawSubClosedPos = 0.26;
    public static double clawSubOpenPos = 0.1;

    //Correct
    public static double clawSubRotateUp = 1;
    public static double clawSubRotateDown = 0.33;

    //Correct
    public static double clawClosedPos = 1;
    public static double clawOpenPos = 0.7;

    //Correct
    public static double clawRotateBucketPos = 0.45;
    public static double clawRotatePickUpPos = 0.75;
    public static double clawRotateSpecimenPos = 0.8;
    public static double clawRotateHangPos = 0.15;// try 0.85


    public static double clawVertPickUpPos = 0.78;
    public static double clawVertBucketPos = 0.55;//correct
    public static double clawVertSpecimenPos = 0.07;//correct
    public static double clawVertHangPos = 0.18;//correct

    //Motor power variables
    public static double powerLeft = 0;
    public static double powerRight = 0;
    public static double powerH = 0;

    //Timer variables
    // For slides
    public static boolean retracting = false;
    public static boolean raising = false;

    // For claw open/close
    public static boolean clawClosing = false;
    public static boolean clawClosingBucket = false;
    public static boolean clawOpeningPickup = false;

    // For raising/lowering claw
    public static boolean clawLowering = false;
    public static boolean clawRaisingBucket = false;
    public static boolean clawRaisingHang = false;

    // For rotating claw
    public static boolean clawRotatingPickUp = false;
    public static boolean clawRotatingSpecimen = false;


    //new variable
    public static boolean clawRotatingHang = false;

    public static boolean clawLowerIngToSpecimen = false;

    // For clawSub open/close
    public static boolean clawSubOpening = false;
    public static boolean clawSubClosing = false;

    // For clawSub rotating
    public static boolean clawSubRotatingUp = false;
    public static boolean clawSubRotatingDown = false;



    //Timer variables
    public static double standardTime = 250;
    public static ElapsedTime matchTimer = new ElapsedTime();
    //for slide PIDs
    public static ElapsedTime timer = new ElapsedTime();
    public static ElapsedTime timerH = new ElapsedTime();
    //for delay between functions
    public static ElapsedTime delayTimer = new ElapsedTime();

    public static double lastError = 0;
    public static double lastErrorH = 0;

    //Mecanum Motor Variables
    public static DcMotor frontRightMotor;
    public static DcMotor frontLeftMotor;
    public static DcMotor backRightMotor;
    public static DcMotor backLeftMotor;

    //Servo variables
    public static Servo clawSubRotate;
    public static Servo clawSub;
    public static Servo clawRotate;
    public static Servo clawVert;
    public static Servo claw;

    //Elevator variables
    public static DcMotorEx leftElevator;
    public static DcMotorEx rightElevator;
    public static DcMotorEx horizontalElevator;

    //DeadZone variables for stick drift
    public static double leftStickDeadZone = 0.1;
    public static double rightStickDeadZone = 0.1;
    public static double speedMultiplier = 1;

}
