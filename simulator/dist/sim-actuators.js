/** Mechanical response to Java actuator commands; contains no autonomous decisions. */
export class SimActuators {
 constructor(game,shoot){this.game=game;this.shoot=shoot;this.reset();}
 reset(){this.lastFeed=-Infinity;}
 apply(o,dt,time){
  const [fl,fr,bl,br]=o.wheelPowers;
  const forward=(fl+fr+bl+br)/4,strafe=(fl-fr-bl+br)/4,clockwise=(fl-fr+bl-br)/4;
  this.game.intakeEnabled=o.intakePower>0;
  const moved=this.game.drive(forward,strafe,-clockwise,dt);
  if(!forward&&!strafe&&!clockwise)this.game.intakeMotion={moving:false,advancing:false};
  // Feeder throughput is a hardware approximation. The Java controller sets continuous power.
  if(o.feederPower>0&&o.shooterPower>0&&time-this.lastFeed>=.5-1e-9){if(this.shoot(o.shooterPower))this.lastFeed=time;}
  this.game.step(dt);return moved;
 }
}
