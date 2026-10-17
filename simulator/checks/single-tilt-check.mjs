import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createInterface} from 'node:readline';
import * as T from '../dist/vendor/three.module.js';
import {compileJava,launchJava} from '../bridge/java-runtime.mjs';
import {SimSensors} from '../dist/sim-sensors.js';
import {SimActuators} from '../dist/sim-actuators.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics,robotAllowed}=await import('../dist/game-physics.js');
const {fencePose,simulate,makeTarget}=await import('../dist/ballistics.js');
const {fireNextBall}=await import('../dist/next-shot.js');
await compileJava();
for(const [team,alliance] of [['RedTeamOne',0],['RedTeamTwo',0],['BlueTeamOne',1],['BlueTeamTwo',1]]){
 const source=readFileSync(new URL(`../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/quantumstorm/Auto/${team}SingleTiltAutonomous.java`,import.meta.url),'utf8');
 const position=`${alliance===0?'RED':'BLUE'}_${team.endsWith('One')?1:2}`;
 assert(source.includes(`AutoStartPosition.${position}`));assert(source.includes('extends SingleTiltAutoBase'));
 assert(source.includes(`name="${team.replace(/([A-Z])/g,' $1').trim()} Single Tilt"`));
 for(const empty of [false,true,'field']){
  const child=launchJava(),lines=createInterface({input:child.stdout}),queue=[],waiters=[];
  lines.on('line',line=>{const m=JSON.parse(line);if(waiters.length)waiters.shift()(m);else queue.push(m);});
  const next=()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>waiters.push(resolve));
  const ready=await next(),send=async m=>{child.stdin.write(JSON.stringify({protocol:1,...m})+'\n');return next();};
  try{
   const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);
   const game=new GamePhysics(field,scene,robot);game.pose=fencePose(alliance);game.syncRobot();
   if(empty){const preloads=new Set(game.inventory);for(const ball of preloads){game.world.removeBody(ball.body);ball.mesh.removeFromParent();}game.balls=game.balls.filter(b=>!preloads.has(b));game.inventory=[];}
   if(empty==='field'){for(const ball of game.balls){game.world.removeBody(ball.body);ball.mesh.removeFromParent();}game.balls=[];for(const flower of game.flowers)flower.queue=[];}
   const sensors=new SimSensors(game);let time=0,seq=0,shots=0,feedSeconds=0,parkAt=null,pickedUp=0,sawIntake=false;
   const actuators=new SimActuators(game,power=>{const fired=fireNextBall(game,{power,angle:66.5,height:6,compression:.15},makeTarget(game.raisedCell(alliance)),{pollen:.5,nectar:.5},simulate);if(fired)shots++;return fired;});
   const read=()=>{const s=sensors.read(time,0,ready);s.cameraConnected=false;s.vision={valid:false,pipeline:0,capturedAt:time,targets:[]};s.ballCount=empty?0:-1;return s;};
   let r=await send({type:'start',runId:team,seq,time,alliance,cameraFree:true,singleTilt:true,shooterPower:.395,sensors:read()});
   while(r.running&&time<30.1){
    assert.equal(r.type,'command');assert(robotAllowed(game.pose));assert(!['WAIT_TILT','WAITING','ERROR','EXTRA_PICKUP'].includes(r.state),'Single tilt only shoots, visits nearest Flower, then parks');
    assert.doesNotMatch(r.status,/→ Garden|INTAKE · GARDEN/,'Single Tilt must not visit Garden');
    if(r.state==='INTAKE')sawIntake=true;
    if(r.state==='PARK'){if(parkAt===null)parkAt=time;assert.equal(r.outputs.shooterPower,0);assert.equal(r.outputs.feederPower,0);}
    if(r.outputs.feederPower>0)feedSeconds+=.02;
    const count=game.inventory.length;actuators.apply(r.outputs,.02,time);if(game.inventory.length>count)pickedUp+=game.inventory.length-count;assert(game.inventory.length<=4);assert(robotAllowed(game.pose));
    time=Number((time+.02).toFixed(8));seq++;r=await send({type:'step',runId:team,seq,time,sensors:read()});
   }
   assert.equal(r.type,'command');assert.equal(r.state,'PARK');assert.equal(r.running,false);
   assert(sawIntake,'Pickup attempt must execute even with measured zero inventory');
   if(empty==='field')assert.equal(pickedUp,0,'Empty field cannot fabricate pollen');
   else{const nearest=game.flowers.find(f=>f.x===(alliance===0?-24:24));assert(nearest.released,'Must touch the actual closest Flower, not the far side Flower');if(!empty)assert(game.inventory.some(b=>b.startZone==='flower'),'Normal run must collect real Flower pollen');}
   assert(parkAt>3&&parkAt<26,'Park begins after pickup, without waiting until 26');assert(time<30);
   assert(feedSeconds>=1.98&&feedSeconds<=2.04,'Exactly one timed feed cycle');assert.equal(shots,empty?0:4);
   assert.equal(game.hives[alliance].tilts,empty?0:1,'Opening four-ball load causes only one physical tilt');
   for(const key of ['forward','strafeRight','turnClockwise','shooterPower','feederPower','intakePower'])assert.equal(r.outputs[key],0);
   assert(Math.hypot(game.pose.x-(alliance===0?-59:59),game.pose.z-(alliance===0?-30:30))<.7,'Actually reaches park');
   console.log('SINGLE TILT',{team,empty,parkAt,time,shots,tilts:game.hives[alliance].tilts});
  }finally{lines.close();child.stdin.end();child.kill();}
 }
}
