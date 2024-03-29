//Allows this file to be used by other files, and for this file to use other files:
package org.firstinspires.ftc.teamcode;
//Importing necessary files:
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//This class contains all the methods and hardware variables you will need to use the robot, and more of each can be added as necessary:
public class RobotMethods extends LinearOpMode {
    //Global variable declaration:
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DistanceSensor distanceSensor;
    //Required to have this method when extending LinearOpMode:
    @Override
    public void runOpMode(){}
    //Declares all of the motors and servos, and can add more if needed:
    public RobotMethods(HardwareMap hardwareMap){
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FRONT LEFT WHEEL NAME");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BACK LEFT WHEEL NAME");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FRONT RIGHT WHEEL NAME");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BACK RIGHT WHEEL NAME");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DISTANCE SENSOR NAME");
    }
    //Moves the robot according to given parameters: (Note: the method runs once and sets the power, it does not stop the robot automatically, the user must do that manually by using move(0,0,0) )
    public void move(double axial, double lateral, double yaw){
        double max;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }
    //Sets the direction of the 4 wheels to be 'forward':
    public void SetDirectionForward() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    //Sets the direction of the 4 wheels to be 'backwards':
    public void SetDirectionBackwards(){
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    //Sets the motors to brake when their power is set to zero:
    public void setZeroBehaviorAll() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //This method moves the robot for a set amount of time depending on the call's arguments (Only use for autonomous):
    //haltDistance is the distance in CM of which the robot will stop if the Distance Sensor detects something closer than the specified distance (Used to stop collisions, and keep in mind that the robot will not continue moving until there is nothing close enough to the distance sensor)
    //Set haltDistance to 0 or less if you do not want to use the feature.
    public void timedMotorMove(double time, double axial, double lateral, double yaw, int haltDistance) {
        double pauseTime = 0;
        for (double startTime = runtime.milliseconds(); runtime.milliseconds() - startTime - pauseTime < time; ) {
            move(axial, lateral, yaw);
            if (haltDistance > 0 && distanceSensor.getDistance(DistanceUnit.CM) < haltDistance) {
                double startPause = runtime.milliseconds();
                while (distanceSensor.getDistance(DistanceUnit.CM) < 10) {
                    move(0, 0, 0);
                }
                pauseTime += runtime.milliseconds() - startPause;
            }
        }
        move(0, 0, 0);
    }
}

