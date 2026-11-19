package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import java.lang.reflect.Proxy;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Desktop checks against the FTC SDK interfaces; no robot or Android context needed. */
public final class ShooterVelocityChecks {
    private static void check(boolean condition,String why) {
        if(!condition)throw new AssertionError(why);
    }

    private static boolean near(double actual,double expected) {
        return Math.abs(actual-expected)<1e-6;
    }

    public static void main(String[] args) {
        double[] velocity={Double.NaN},feedPower={Double.NaN};
        int[] powerCalls={0};
        DcMotor.RunMode[] mode={null};
        MotorConfigurationType originalType=new MotorConfigurationType();
        originalType.setTicksPerRev(2786);
        originalType.setGearing(99.5);
        originalType.setMaxRPM(60);
        MotorConfigurationType[] motorType={originalType};
        DcMotorEx motor=(DcMotorEx)Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class},(proxy,method,values)->{
                    switch(method.getName()) {
                        case "getMotorType": return motorType[0];
                        case "setMotorType": motorType[0]=(MotorConfigurationType)values[0];return null;
                        case "setMode": mode[0]=(DcMotor.RunMode)values[0];return null;
                        case "setPower": powerCalls[0]++;return null;
                        case "setVelocity":
                            velocity[0]=(double)values[0];
                            check(velocity[0]==0 || values.length==2 && values[1]==AngleUnit.DEGREES,
                                    "Nonzero velocity must specify angular units, not encoder ticks");
                            return null;
                        case "getVelocity":
                            check(values.length==1 && values[0]==AngleUnit.DEGREES,"Read angular velocity");
                            return 18000.0; // Actual speed differs from the requested speed.
                        default: return null;
                    }
                });
        CRServo feeder=(CRServo)Proxy.newProxyInstance(CRServo.class.getClassLoader(),
                new Class<?>[]{CRServo.class},(proxy,method,values)->{
                    if(method.getName().equals("setPower"))feedPower[0]=(double)values[0];
                    return null;
                });
        Shooter shooter=new Shooter(motor,feeder);
        check(mode[0]==DcMotor.RunMode.RUN_USING_ENCODER,"Encoder velocity mode on INIT");
        check(velocity[0]==0 && feedPower[0]==0,"INIT stops both mechanisms");
        shooter.spinUp();
        check(near(shooter.getTargetRpm(),2940) && near(velocity[0],17640),"Default 2940 RPM converted to deg/s");
        check(shooter.getMeasuredRpm()==3000,"Telemetry reports measured RPM, not requested speed");
        shooter.apply(.50,0);
        check(near(shooter.getTargetRpm(),3000) && near(velocity[0],18000),"One multiplier step adds 60 RPM");
        shooter.apply(2,0);
        check(shooter.getTargetRpm()==6000 && velocity[0]==36000,"Positive target is bounded");
        shooter.apply(-2,1);
        check(shooter.getTargetRpm()==-6000 && velocity[0]==-36000 && feedPower[0]==0,
                "Negative target is bounded and does not feed");
        for(double invalid:new double[]{Double.NaN,Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY,0}) {
            shooter.apply(invalid,1);
            check(shooter.getTargetRpm()==0 && velocity[0]==0 && feedPower[0]==0,
                    "Invalid or zero speed disables shooter and feeding");
        }
        shooter.spinUp();
        shooter.stop();
        check(shooter.getTargetRpm()==0 && velocity[0]==0 && feedPower[0]==0,"STOP clears velocity and feed");
        check(powerCalls[0]==0,"Shooter motor never receives a power command, including INIT and STOP");
        System.out.println("Shooter velocity, RPM conversion/readback, bounds, invalid input, feed guard and STOP checks passed.");
    }
}
