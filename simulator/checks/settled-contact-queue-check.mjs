import assert from 'node:assert/strict';
import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics}=await import('../dist/game-physics.js');
const {makeTarget,simulate}=await import('../dist/ballistics.js');
const {previewNextBall,fireNextBall,shotColor}=await import('../dist/next-shot.js');
const setup=()=>{const f=buildField(),r=buildRobot(),s=new T.Scene();s.add(f,r);const g=new GamePhysics(f,s,r);for(const b of g.balls){g.world.removeBody(b.body);b.mesh.removeFromParent();}g.balls=[];g.inventory=[];g.syncInventory();g.pose={x:35,z:0,theta:0};g.syncRobot();return g;};
const settings={power:.4,angle:71.5,height:6,compression:.15},ratios={pollen:.5,nectar:.5};
for(const types of [['pollen','pollen','red'],['red','blue','pollen'],['pollen','red','blue']]){
 const g=setup();types.forEach(t=>g.collect(g.createBall(t,[0,2,0])));const target=makeTarget(g.raisedCell());let previous=null;
 for(const type of types){const frame=previewNextBall(g,settings,target,ratios,simulate),head=g.nextBall;
  assert.equal(frame.shot.ballId,head.body.id);assert.equal(frame.settings.ballId,head.body.id);assert.equal(frame.shot.ballColor,type);assert.equal(frame.shot.type,type==='pollen'?'pollen':'nectar');assert.equal(shotColor(frame.shot.ballColor),type==='pollen'?'#f5d650':type==='red'?'#ef6961':'#629cef');
  if(previous&&previous.type===frame.shot.type)assert.equal(frame.shot.apexHeight,previous.apexHeight,'Equal ball types keep the same curve');
  if(previous&&previous.type!==frame.shot.type)assert.notEqual(frame.shot.apexHeight,previous.apexHeight,'Change curve exactly when the next-to-fire type changes');
  assert(fireNextBall(g,settings,target,ratios,simulate));previous=frame.shot;g.elapsed+=.3;
 }
 assert.equal(previewNextBall(g,settings,target,ratios,simulate),null);
}
const inside=(g,b)=>{const c=Math.cos(g.pose.theta),s=Math.sin(g.pose.theta),x=b.body.position.x/.0254-g.pose.x,z=b.body.position.z/.0254-g.pose.z;return Math.abs(c*x-s*z)<8.75&&s*x+c*z>-9.05&&s*x+c*z<8.055;};
for(const theta of [0,.6,Math.PI/2])for(const dt of [1/120,1/60,.04])for(const face of ['side','rear','front']){
 const g=setup();g.pose.theta=theta;g.syncRobot();const c=Math.cos(theta),s=Math.sin(theta),local=face==='side'?[12,0]:face==='rear'?[0,12]:[0,-13];
 const b=g.createBall('pollen',[g.pose.x+c*local[0]+s*local[1],1.4,g.pose.z-s*local[0]+c*local[1]]);
 // Use a genuinely settled/sleeping body, unlike the old side-contact test.
 for(let i=0;i<240;i++)g.step(1/120);b.body.sleep();assert.equal(b.body.sleepState,2);const initial=b.body.position.clone();
 for(let time=0;time<.38;time+=dt){g.drive(face==='front'?1:face==='rear'?-1:0,face==='side'?1:0,0,dt);g.step(dt);assert(b.held||!inside(g,b),`${face} contact must not pass through the chassis`);}
 if(face==='front'){assert(b.held);assert.equal(g.inventory.length,1);}else{assert(!b.held);assert(b.body.position.distanceTo(initial)>.0254*4,`${face} must push settled ball`);}
}
// With four already loaded, front contact pushes instead of accepting a fifth.
const full=setup();for(let i=0;i<4;i++)full.collect(full.createBall('pollen',[0,2,0]));const fifth=full.createBall('red',[35,1.8,-13]);fifth.body.sleep();for(let i=0;i<30;i++){full.drive(1,0,0,1/60);full.step(1/60);}assert(!fifth.held);assert.equal(full.inventory.length,4);assert(!inside(full,fifth));
console.log('P→P→N / N→N→P / P→N→N queue snapshots and colors; sleeping-ball side/rear pushing and front pickup at 3 frame rates and 3 headings; full intake pushes passed.');
