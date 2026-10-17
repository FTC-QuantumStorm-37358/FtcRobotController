package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Team One Single Tilt",group="Quantum Storm",preselectTeleOp="Quantum Storm Two Driver")
public final class RedTeamOneSingleTiltAutonomous extends SingleTiltAutoBase {
    @Override protected AutoStartPosition startPosition(){return AutoStartPosition.RED_1;}
}
