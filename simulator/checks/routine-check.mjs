import assert from 'node:assert/strict';
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
for(const alliance of [0,1]){
 const child=launchJava(),lines=createInterface({input:child.stdout}),queue=[],waiters=[];
 lines.on('line',line=>{const m=JSON.parse(line);if(waiters.length)waiters.shift()(m);else queue.push(m);});
 const next=()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>waiters.push(resolve));
 const ready=await next();
 const send=async m=>{child.stdin.write(JSON.stringify({protocol:1,...m})+'\n');return next();};
 try{
  const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);
  const game=new GamePhysics(field,scene,robot);game.pose=fencePose(alliance);game.syncRobot();
  const camera=new SimSensors(game);let time=0,seq=0,shots=0,lastStatus='',lastCount=4,parkStartedAt=null;
  const pickups=[],firedSources=[],tiltTimes=[];let lastTilts=0,tiltsAtPark=null,lastFlowerShotAt=null;
  const actuators=new SimActuators(game,power=>{const source=game.nextBall?.startZone;const fired=fireNextBall(game,{power,angle:66.5,height:6,compression:.15},makeTarget(game.raisedCell(alliance)),{pollen:.5,nectar:.5},simulate);if(fired){shots++;firedSources.push(source);if(source==='flower')lastFlowerShotAt=time;}return fired;});
  let r=await send({type:'start',runId:'routine'+alliance,seq,time,alliance,shooterPower:.395,sensors:camera.read(time,0,ready)});
  while(r.running&&time<30.1){
   assert.equal(r.type,'command');assert(robotAllowed(game.pose));
   assert.notEqual(r.state,'EXTRA_PICKUP','No optional pickup state may run');
   if(r.state==='WAITING')for(const key of ['forward','strafeRight','turnClockwise','shooterPower','feederPower','intakePower'])assert.equal(r.outputs[key],0,'Completed routine must hold still until parking');
   if(r.state==='PARK'){
    if(parkStartedAt===null){parkStartedAt=time;tiltsAtPark=game.hives[alliance].tilts;}
    assert(time>=26,'Parking must not start before 26 seconds');
    for(const key of ['shooterPower','feederPower','intakePower'])assert.equal(r.outputs[key],0,'Parking cancels scoring');
   }
   const short=r.status.split(' · ')[0];
   if(short!==lastStatus){console.log(alliance,time,short);lastStatus=short;}
   const o=r.outputs;actuators.apply(o,.02,time);
   if(game.hives[alliance].tilts>lastTilts){tiltTimes.push(time+.02);lastTilts=game.hives[alliance].tilts;}
   assert(game.inventory.length<=4);assert(robotAllowed(game.pose));
   if(game.inventory.length>lastCount)pickups.push(...game.inventory.slice(lastCount).map(b=>b.startZone));lastCount=game.inventory.length;
   time=Number((time+.02).toFixed(8));seq++;
   r=await send({type:'step',runId:'routine'+alliance,seq,time,sensors:camera.read(time,o.pipeline,ready)});
  }
  console.log('RESULT',{alliance,parkStartedAt,time,shots,pickups,lastFlowerShotAt,tiltTimes,status:r.status,pose:game.pose,hive:game.hives[alliance].counts,tilts:game.hives[alliance].tilts});
  assert.equal(parkStartedAt,26,'Parking must begin at the 26-second trigger');
  assert.equal(r.state,'PARK');
  assert.equal(r.running,false);
  assert.deepEqual(pickups,[...Array(4).fill('garden'),...Array(4).fill('flower')],'Only the four Garden and four Flower balls may be picked up');
  assert.equal(game.inventory.length,0,'No extra inventory after the final Flower batch');
  assert.deepEqual(firedSources,[...Array(4).fill('robot'),...Array(4).fill('garden'),...Array(4).fill('flower')],'All three batches must actually fire in order');
  assert(lastFlowerShotAt<26,'All Flower shots must finish before parking');
  assert(tiltsAtPark>=2,'Flower shots must cause a second completed physical tilt before parking');
  assert.match(r.status,/Flower tilt confirmed/,'Java must independently confirm the final tilt from measured AprilTags');
  assert(time<30,'Parking must finish before 30 seconds');
  for(const key of ['forward','strafeRight','turnClockwise','shooterPower','feederPower','intakePower'])assert.equal(r.outputs[key],0,'Park disables every actuator');
  const park={x:alliance===0?-59:59,z:alliance===0?-30:30};
  assert(Math.hypot(game.pose.x-park.x,game.pose.z-park.z)<.7,'Robot must actually reach the alliance Loading Zone park pose');
 }finally{lines.close();child.stdin.end();child.kill();}
}
