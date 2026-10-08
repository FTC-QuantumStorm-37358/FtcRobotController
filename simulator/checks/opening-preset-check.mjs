import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');const {GamePhysics}=await import('../dist/game-physics.js');const {simulate,makeTarget,fencePose}=await import('../dist/ballistics.js');
for(const alliance of [0,1]){const f=buildField(),r=buildRobot(),s=new T.Scene();s.add(f,r);const g=new GamePhysics(f,s,r);g.pose=fencePose(alliance);g.syncRobot();const shot=simulate({power:.395,angle:66.5,height:6,transfer:.5,compression:.15,type:'pollen',pose:g.pose},makeTarget(g.raisedCell(alliance)));assert.equal(shot.status,'entry');assert(shot.entry.verticalSpeed<0);assert(shot.entry.margin>6);assert(Math.abs(shot.hoodSweep*180/Math.PI-124)<1e-10);assert(g.shoot(shot));let retained=0;for(let i=0;i<360;i++){g.step(1/120);retained=Math.max(retained,...g.hives[alliance].counts.map(c=>c.pollen));}assert(retained>=1,'Starting shot must physically enter and stay inside the hive');}
assert.match(readFileSync('dist/shot.js','utf8'),/power:\.395,angle:66\.5/);assert.match(readFileSync('dist/index.html','utf8'),/id="angle"[^>]*value="66\.5"/);assert.equal(readFileSync('dist/index.html','utf8'),readFileSync('dist/shot.html','utf8'));
console.log('66.5-degree / .395 initial preset; 124-degree hood; descending opening clearance and retained physical shots at both alliance fence presets passed.');
