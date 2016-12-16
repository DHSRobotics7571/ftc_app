package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Autonomous - RED", group="Autonomous")
public class autoRed extends OpMode {
    //instantiate objects
    private DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    private OpticalDistanceSensor ODSright, ODSleft;
    private ColorSensor color;
    private CRServo beaconpusher;
    //instantiate logic objects
    private int robostate = 1;
    //the closest the robot can be to the vision target
    private double thresholdmax = 0.1;
    //the farthest away the robot can be to the vision target
    private double thresholdmin = 0.1;
    @Override
    public void init(){
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");

        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");

        beaconpusher = hardwareMap.crservo.get("beacon");

        color = hardwareMap.colorSensor.get("color");
    }
    @Override
    public void loop(){
        switch(robostate){
            case 1:
                if(squareUp()){
                    stopMotors();
                    robostate++;
                }
                break;
            case 2:
                /*
                    You are squared up with the beacon.
                    What will you do?
                    You have a servo beacon pusher, and a color sensor.
                 */
                break;
            default:
                break;
        }

    }
    @Override
    public void stop(){

    }

    //square robot with the wall to push the beacons
    public boolean squareUp(){
        //initial check to see if close to vision target
        if(ODSright.getRawLightDetected()<thresholdmin&&ODSleft.getRawLightDetected()<thresholdmin){
            //waay too far, go closer.
            strafe(0.1);
        }else{
            //check to see if one or both sensors are within the thresholds
            if(ODSright.getRawLightDetected()>thresholdmin && ODSleft.getRawLightDetected()>thresholdmin){
                //both sensors are within the minimum distance - check to see if squared.
                if(ODSright.getRawLightDetected()<thresholdmax && ODSleft.getRawLightDetected()<thresholdmax){
                    //both sensors within both thresholds, we are squared up.
                    return true;
                }else{
                    if(ODSright.getRawLightDetected()>thresholdmax && ODSleft.getRawLightDetected()>thresholdmax){
                        //both sensors too close, back up.
                        strafe(-0.1);
                    } else{

                    }
                }

            } else{
                //one of the sensors is not within thresholds
                if(ODSright.getRawLightDetected()<thresholdmin){
                    //right sensor isnt within min distance - fix it
                    turn(-0.1);
                }else{
                    //left sensor isnt within min distance - fix it
                    turn(0.1);
                }
            }
        }
        //

        //return whether or not we are squared up

        return false;
    }

    public void stopMotors(){
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
    }
    /*
        If power is positive, goes forward
        If power is negative, goes backwards
     */
    public void setThrottle(double power){
        motorRightBack.setPower(power);
        motorRightFront.setPower(-power);
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(-power);
    }
    /*
        If power is positive, turns right
        If power is negative, turns left
     */
    public void turn(double power){
        motorRightBack.setPower(-power);
        motorRightFront.setPower(-power);
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(power);
    }
    /*
        If power is positive, strafes right
        If power is negative, strafes left
     */
    public void strafe(double power){
        motorRightBack.setPower(power);
        motorRightFront.setPower(-power);
        motorLeftBack.setPower(power);
        motorLeftFront.setPower(-power);
    }

}