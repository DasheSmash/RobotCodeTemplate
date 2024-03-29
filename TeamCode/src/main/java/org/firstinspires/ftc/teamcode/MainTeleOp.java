//Allows for this file to use other files:
package org.firstinspires.ftc.teamcode;
//Imports necessary files:
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Main TeleOp", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {
    //Code that will run when the user presses 'INIT':
    @Override
    public void runOpMode() {
        //Variable declaration, initialization, and instantiation:
        ElapsedTime runtime = new ElapsedTime();
        RobotMethods RMO = new RobotMethods(hardwareMap);
        //Sets the robot's wheels to go 'backwards', and for the robot's motors to brake when at 0 power:
        RMO.SetDirectionBackwards();
        RMO.setZeroBehaviorAll();
        //Variables to move the robot:
        double axial = 0.0;
        double lateral = 0.0;
        double yaw = 0.0;
        //Variable to adjust the speed of the robot:
        double driveMultiplier = 0.5;
        //Waits for the play button to be pressed:
        waitForStart();
        runtime.reset();
        //Repeatedly runs after the play button is pressed:
        while(opModeIsActive()) {
            //Pressing start on either of the controllers will stop the code:
            if (gamepad1.back || gamepad2.back){terminateOpModeNow();}
            //Pressing start will stop the robot:
            while(gamepad1.start || gamepad2.start) {
                RMO.move(0,0,0);
            }
            //Changes where the robot will go according to the stick directions and the speed setting:
            axial = driveMultiplier * (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier * (gamepad1.left_stick_x);
            yaw = driveMultiplier * (-gamepad1.right_stick_x);
            //Determines speed setting:
            if (gamepad1.left_trigger > 0.1f){driveMultiplier = 0.25;}
            else if (gamepad1.right_trigger > 0.1f){driveMultiplier = 1;}
            else{driveMultiplier = 0.5;}
            //Implements the changes to the robot's position from other parts of the code:
            RMO.move(axial,lateral,yaw);
            //Sends data back to the driver's station:
            telemetry.addData("Current centimeters from distance sensor: ", RMO.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}