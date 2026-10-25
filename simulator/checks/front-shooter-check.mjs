import assert from 'node:assert/strict';
import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics,robotAllowed}=await import('../dist/game-physics.js');
const {launcher,SHOOTER_REAR_OFFSET,REAR_BALL_GAP,REAR_EDGE,WHEEL,shooterHeading,fencePose,simulate,makeTarget}=await import('../dist/ballistics.js');
for(const theta of [0,.6,Math.PI/2,Math.PI,4.7]){const pose={x:35,z:25,theta},front=new T.Vector3(-Math.sin(theta),0,-Math.cos(theta)),rear=front.clone().negate();for(const diameter of [2.8,3.6]){const l=launcher(71.5,6,.15,pose,diameter);assert(l.forward.distanceTo(rear)<1e-12);assert(Math.abs(l.center.clone().sub(new T.Vector3(pose.x,6,pose.z)).dot(rear)-SHOOTER_REAR_OFFSET)<1e-10);assert(Math.abs(REAR_EDGE-SHOOTER_REAR_OFFSET-WHEEL/2-REAR_BALL_GAP)<1e-10);assert(l.origin.clone().sub(l.center).dot(rear)<0);}}
for(const [dx,dz] of [[1,2],[-5,2],[-1,-3],[2,-1]]){const theta=shooterHeading(dx,dz),forward=launcher(70,6,.15,{x:0,z:0,theta}).forward;assert(forward.dot(new T.Vector3(dx,0,dz).normalize())>1-1e-12);}
const f=buildField(),r=buildRobot(),s=new T.Scene();s.add(f,r);const g=new GamePhysics(f,s,r);
for(const alliance of [0,1]){const pose=fencePose(alliance);assert(robotAllowed(pose));assert(Math.abs(pose.z)+9.05<72);const shot=simulate({power:.395,angle:66.5,height:6,compression:.15,transfer:.5,type:'pollen',pose},makeTarget(g.raisedCell(alliance)));assert.equal(shot.status,'entry');assert(new T.Vector3(shot.vx,0,shot.vz).dot(new T.Vector3(-Math.sin(pose.theta),0,-Math.cos(pose.theta)))<0);}
console.log('Rear-facing launch velocity at five headings; rear aim; legal front-at-fence starts and descending Hive entries passed.');
