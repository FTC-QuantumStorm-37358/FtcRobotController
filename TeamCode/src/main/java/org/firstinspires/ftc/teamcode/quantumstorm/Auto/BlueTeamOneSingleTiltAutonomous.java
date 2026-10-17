package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Team One Single Tilt",group="Quantum Storm",preselectTeleOp="Quantum Storm Two Driver")
public final class BlueTeamOneSingleTiltAutonomous extends SingleTiltAutoBase {
    @Override protected AutoStartPosition startPosition(){return AutoStartPosition.BLUE_1;}


}
