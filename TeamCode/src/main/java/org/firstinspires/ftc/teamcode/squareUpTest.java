package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Autonomous - RED", group="Autonomous")
public class squareUpTest extends OpMode {
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
    private long time;
    @Override
    public void init(){
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");

        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsleft");
    }
    @Override
    public void loop(){
        squareUp();

    }
    @Override
    public void stop(){
        setThrottle(0);
    }
    public boolean push(){
        if(time == 0){
            time = System.currentTimeMillis();
            beaconpusher.setPower(1);
        }
        else{
            if(System.currentTimeMillis()-time>=4000){
                beaconpusher.setPower(-1);
            }
            if(System.currentTimeMillis()-time>=8000){
                beaconpusher.setPower(0);
                return true;
            }
        }
        return false;
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
                    setThrottle(0);
                    return true;
                }else{
                    if(ODSright.getRawLightDetected()>thresholdmax && ODSleft.getRawLightDetected()>thresholdmax){
                        //both sensors too close, back up.
                        strafe(-0.1);
                    } else{
                        //one sensor too close, adjust that one.
                        if(ODSright.getRawLightDetected()>thresholdmax){
                            //god dammit right sensor you arent squared - fix it
                            turn(0.1);
                        }else{
                            //god dammit left sensor you arent squared - fix it
                            turn(-0.1);
                        }

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
        //currently adjusting position, isn't squared up yet, return false
        return false;
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
