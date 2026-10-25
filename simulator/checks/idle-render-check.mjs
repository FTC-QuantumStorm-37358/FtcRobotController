import assert from 'node:assert/strict';
import {DemandLoop} from '../dist/demand-loop.js';
import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');const {GamePhysics}=await import('../dist/game-physics.js');const {simulate,makeTarget}=await import('../dist/ballistics.js');
const f=buildField(),r=buildRobot(),s=new T.Scene();s.add(f,r);const g=new GamePhysics(f,s,r);
assert(g.hasActivity());for(let i=0;i<1200;i++)g.step(1/60);assert(!g.hasActivity(),'Starting balls must settle so idle rendering can stop');
let draws=0,callbacks=new Map(),counter=0,active=false;
const request=fn=>{callbacks.set(++counter,fn);return counter;},cancel=id=>callbacks.delete(id);
const loop=new DemandLoop(()=>{draws++;return active;},request,cancel);
const tick=()=>{const [id,fn]=callbacks.entries().next().value;callbacks.delete(id);fn(100);};
loop.wake();loop.wake();assert.equal(callbacks.size,1);tick();assert.equal(draws,1);assert.equal(callbacks.size,0,'Idle scene must schedule zero further frames');
// Time passing alone never causes another draw; interaction wakes one immediately.
assert.equal(draws,1);loop.wake();active=true;tick();assert.equal(callbacks.size,1);active=false;tick();assert.equal(callbacks.size,0);
loop.wake();loop.pause();assert.equal(callbacks.size,0,'Hidden tab must cancel scheduled rendering');
const shot=simulate({power:.4,angle:71.5,height:6,transfer:.5,compression:.15,pose:g.pose,type:'pollen'},makeTarget(g.raisedCell()));assert(g.shoot(shot));assert(g.hasActivity(),'Shooting must wake physics');for(let i=0;i<1800;i++)g.step(1/60);assert(!g.hasActivity(),'Shot must return to idle once settled');
const ff=buildField(),rr=buildRobot(),ss=new T.Scene();ss.add(ff,rr);const full=new GamePhysics(ff,ss,rr),flower=full.flowers[0];full.pose={x:flower.x+flower.direction.x*20,z:flower.z+flower.direction.z*20,theta:Math.atan2(flower.direction.x,flower.direction.z)};full.syncRobot();
for(let i=0;i<110;i++){full.drive(1,0,0,1/120);full.step(1/120);}assert(flower.released);assert(full.hasActivity(),'Delayed flower release must keep physics running');for(let i=0;i<1800;i++)full.step(1/60);assert.equal(flower.queue.length,0);assert.equal(full.inventory.length,4);for(let i=0;i<7200&&full.hasActivity();i++)full.step(1/60);assert(!full.hasActivity(),'Full-intake release must eventually return to idle');
console.log('Initial scene, shot, and flower release settle to zero activity; demand loop schedules no idle frames, wakes on interaction, coalesces requests and cancels when hidden passed.');
