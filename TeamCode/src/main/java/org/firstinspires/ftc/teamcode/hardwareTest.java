package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="HardwareTest", group="Autonomous")
public class hardwareTest extends OpMode implements SensorEventListener {
    //instantiate objects
    private DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    private SensorManager senSensorManager;
    private Sensor senAccelerometer;
    private float x=0,y=0,z=0;
    private float xc=0,yc=0,zc=0;
    private double time = 0;
    @Override
    public void init(){
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");
    }

    @Override
    public void loop(){
        //Mecanum wheel drive - Vector Addition and subtraction
        float[] motorVals = {0,0,0,0};
        //Right Front
        motorVals[0] = -gamepad1.left_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x;
        //Right Back
        motorVals[1] = -gamepad1.left_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x;
        //Left Front
        motorVals[2] = -gamepad1.left_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x;
        //Left Back
        motorVals[3] = -gamepad1.left_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x;
        //Adjust range to that allowed by DcMotors
        motorVals = map(motorVals,-1,1);
        //Set power to motors
        motorRightFront.setPower(motorVals[0]);
        motorRightBack.setPower(motorVals[1]);
        motorLeftFront.setPower(motorVals[2]);
        motorLeftBack.setPower(motorVals[3]);


        //check accelrometer values
        telemetry.addData("Gyroscope (raw)","X:"+x+"Y:"+y+"Z:"+z);
        telemetry.addData("net change","X:"+xc+"Y:"+yc+"Z:"+zc);
    }
    @Override
    public void stop(){

    }
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        Sensor mySensor = sensorEvent.sensor;

        if (mySensor.getType() == Sensor.TYPE_GYROSCOPE) {
            if(time == 0)
                time = System.currentTimeMillis();
            else{
                xc+=x*((System.currentTimeMillis()-time)/1000.0);
                yc+=y*((System.currentTimeMillis()-time)/1000.0);
                zc+=z*((System.currentTimeMillis()-time)/1000.0);
                time = System.currentTimeMillis();
            }

            x = sensorEvent.values[0];
            y = sensorEvent.values[1];
            z = sensorEvent.values[2];
        }
    }
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
    //you dont need to know what this does.
    public static float[] map(float[] a, double minR, double maxR){
        float min=Float.MAX_VALUE;
        float max=Float.MIN_VALUE;
        for(int i = 0;i<=3;i++){
            if(a[i] < min){
                min = a[i];
            }
            if(a[i] > max){
                max = a[i];
            }
        }
        if(min>=-1&&max<=1){
            return a;
        }
        double scalemin = Math.abs(minR/min);
        double scalemax = Math.abs(minR/max);
        double finalscale;

        if(scalemin<scalemax)
            finalscale = scalemin;
        else
            finalscale = scalemax;
        for(int i = 0;i<=3;i++){
            a[i] = a[i]*(float)finalscale;
        }

        return a;
    }

}