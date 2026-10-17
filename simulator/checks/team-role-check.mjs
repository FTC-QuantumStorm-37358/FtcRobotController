import assert from 'node:assert/strict';
import {createInterface} from 'node:readline';
import * as T from '../dist/vendor/three.module.js';
import {compileJava,launchJava} from '../bridge/java-runtime.mjs';
import {SimSensors} from '../dist/sim-sensors.js';
import {SimActuators} from '../dist/sim-actuators.js';
import {AUTONOMOUS_MODES,placeAutonomousRobot} from '../dist/autonomous-modes.js';
import {fireNextBall} from '../dist/next-shot.js';
import {simulate,makeTarget} from '../dist/ballistics.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics,robotAllowed}=await import('../dist/game-physics.js');
await compileJava();
for(const mode of AUTONOMOUS_MODES.filter(m=>m.teamRole)){
 const child=launchJava(),lines=createInterface({input:child.stdout}),queue=[],waiters=[];
 lines.on('line',line=>{const m=JSON.parse(line);if(waiters.length)waiters.shift()(m);else queue.push(m);});
 const next=()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>waiters.push(resolve));
 const ready=await next(),send=async m=>{child.stdin.write(JSON.stringify({protocol:1,...m})+'\n');return next();};
 try{
  const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);
  const game=new GamePhysics(field,scene,robot);placeAutonomousRobot(game,mode);
  assert(robotAllowed(game.pose),'Legal start');
  if(mode.teamRole===2){assert.equal(game.pose.z,mode.alliance===0?-6:6);assert(Math.abs(game.pose.z)<10,'Start is between Loading Zone and side Flower, not in the Loading Zone');}
  const stock=game.balls.filter(b=>b.storedOutside);assert.equal(stock.length,10);assert(stock.every(b=>!b.body.world&&Math.abs(b.body.position.x)/.0254>72));
  assert.equal(game.hives[mode.alliance].tilts,mode.teamRole===2?1:0,'Partner tilt must be physically complete at Team 2 start');
  for(const side of [-1,1])assert.equal(game.balls.filter(b=>b.startZone==='garden'&&Math.sign(b.body.position.x)===side).length,4);
  const sources=[],sensors=new SimSensors(game,{enabled:false});
  const actuators=new SimActuators(game,power=>{
   const source=game.nextBall?.startZone;
   const ok=fireNextBall(game,{power,angle:66.5,height:6,compression:.15},makeTarget(game.raisedCell(mode.alliance)),{pollen:.5,nectar:.5},simulate);
   if(ok)sources.push(source);return ok;
  });
  let time=0,seq=0,parkAt=null,last='';const pickups=new Set();
  const read=pipeline=>{const s=sensors.read(time,pipeline,ready);s.cameraConnected=false;return s;};
  let r=await send({type:'start',runId:mode.id,seq,time,alliance:mode.alliance,cameraFree:true,teamRole:mode.teamRole,shooterPower:.395,sensors:read(0)});
  while(r.running&&time<30.1){
   assert.equal(r.type,'command');assert(!['ERROR','SAFETY_HOLD'].includes(r.state),r.status);
   const phase=r.status.split(' · ')[0];if(phase!==last){console.log(mode.id,time,phase);last=phase;}
   if(r.state==='PARK'&&parkAt===null){parkAt=time;if(mode.teamRole===2)assert.equal(game.hives[mode.alliance].tilts,2,'Second physical tilt completes before leaving to park');}
   actuators.apply(r.outputs,.02,time);assert(robotAllowed(game.pose),'Collision-free route');
   for(const b of game.inventory)if(b.startZone==='garden')pickups.add(b);
   time=Number((time+.02).toFixed(8));seq++;
   r=await send({type:'step',runId:mode.id,seq,time,sensors:read(r.outputs.pipeline)});
  }
  console.log('TEAM ROLE RESULT',{mode:mode.id,time,parkAt,shots:game.shots,pickups:pickups.size,tilts:game.hives[mode.alliance].tilts,status:r.status,pose:game.pose});
  assert.equal(r.state,'PARK');assert(!r.running&&time<30,'Park before deadline');
  assert.deepEqual(sources,[...Array(4).fill('robot'),...Array(mode.teamRole===2?4:0).fill('garden')]);
  assert.equal(pickups.size,mode.teamRole===2?4:0);
  assert(stock.every(b=>!b.held&&!b.body.world&&Math.abs(b.body.position.x)/.0254>72),'Unused nectar remains outside throughout autonomous');
  assert.equal(game.hives[mode.alliance].tilts,mode.teamRole===2?2:1,'Physical tilt, not state label');
  assert(!sources.includes('flower'),'Role routines must use only preloads and Garden pollen');
  assert(Math.hypot(game.pose.x-(mode.alliance===0?-59:59),game.pose.z-(mode.alliance===0?-30:30))<.7,'Actual park pose');
  for(const k of ['forward','strafeRight','turnClockwise','shooterPower','feederPower','intakePower'])assert.equal(r.outputs[k],0);
 }finally{lines.close();child.stdin.end();child.kill();}
}
