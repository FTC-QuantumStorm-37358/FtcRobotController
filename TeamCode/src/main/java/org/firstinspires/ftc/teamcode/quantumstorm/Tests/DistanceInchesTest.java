package org.firstinspires.ftc.teamcode.quantumstorm.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/** Measurement only: no joystick drive and no time-to-distance guesses. */
@TeleOp(name="Test: Distance Inches (motors OFF)",group="Quantum Storm Test")
public final class DistanceInchesTest extends LinearOpMode {
    @Override public void runOpMode(){
        MecanumDrive drive=new MecanumDrive(hardwareMap);
        try{
            PinpointOdometry odometry=new PinpointOdometry(hardwareMap,new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0));
            telemetry.addLine("MOTORS OFF. Keep still for INIT. After PLAY, press A and push exactly 24 inches straight.");telemetry.update();
            waitForStart();
            boolean anchored=false,lastA=false;double originX=0,originY=0,previousX=0,previousY=0,path=0;
            while(opModeIsActive()){
                drive.stop();
                try{
                    odometry.update();
                    if(odometry.isValid()){
                        double x=odometry.getX(),y=odometry.getY();
                        if(!anchored||gamepad1.a&&!lastA){originX=previousX=x;originY=previousY=y;path=0;anchored=true;}
                        path+=Math.hypot(x-previousX,y-previousY);previousX=x;previousY=y;
                        double displacement=Math.hypot(x-originX,y-originY);
                        telemetry.addData("Straight-line distance (in)","%.2f",displacement);
                        telemetry.addData("Accumulated path (in)","%.2f",path);
                        telemetry.addData("Delta X / Y (in)","%.2f / %.2f",x-originX,y-originY);
                        telemetry.addData("For an actual 24-inch move",Math.abs(displacement-24)<=1?"PASS within 1 inch":"Reading should be 24 inches; verify pod type, wiring and direction");
                    }else{anchored=false;telemetry.addLine("Odometry invalid: fix Pinpoint before autonomous.");}
                    odometry.addTelemetry(telemetry);
                }catch(RuntimeException e){anchored=false;telemetry.addData("Sensor fault",e.getMessage());}
                lastA=gamepad1.a;telemetry.addLine("A = zero measurement. No stick input drives motors. Use a tape measure.");telemetry.update();idle();
            }
        }finally{drive.stop();}
    }
}
