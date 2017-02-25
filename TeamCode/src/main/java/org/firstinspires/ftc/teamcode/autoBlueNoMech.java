package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Autonomous - kys", group="Autonomous")
@Disabled
public class autoBlueNoMech extends OpMode {
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
            case 0:
                squareUp();
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
                push();
                setThrottle(.3);

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

        return false;
    }
    public void push() {
        long tim = System.currentTimeMillis();
        beaconpusher.setPower(.1);
        if (tim >= 4000) {
            beaconpusher.setPower(-.1);
            if (tim >= 8000){
                beaconpusher.setPower(0);
            }
        }

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

}