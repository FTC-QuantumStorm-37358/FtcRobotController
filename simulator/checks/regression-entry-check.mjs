import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics}=await import('../dist/game-physics.js');
const {makeTarget,simulate}=await import('../dist/ballistics.js');
const {fireNextBall}=await import('../dist/next-shot.js');
const setup=()=>{const f=buildField(),r=buildRobot(),s=new T.Scene();s.add(f,r);return new GamePhysics(f,s,r);};
assert.equal(readFileSync('dist/index.html','utf8'),readFileSync('dist/shot.html','utf8'));
assert.match(readFileSync('dist/index.html','utf8'),/src="\.\/shot.js(?:\?[^"]*)?"/);
const g=setup(),head=g.nextBall;
assert(fireNextBall(g,{power:.4,angle:71.5,height:6,compression:.15},makeTarget(g.raisedCell()),{pollen:.5,nectar:.5},simulate));
assert.equal(g.inventory.length,3);assert.equal(head.body.world,g.world);assert(head.body.velocity.y>0);
// Actual repeated drive commands reach each movement blocker and release all four balls.
for(let i=0;i<4;i++){
 const game=setup(),f=game.flowers[i];
 const theta=Math.atan2(f.direction.x,f.direction.z);
 game.pose={x:f.x+f.direction.x*20,z:f.z+f.direction.z*20,theta};game.syncRobot();
 for(let k=0;k<90;k++){game.drive(1,0,0,1/120);game.step(1/120);}
 assert(f.released,`Flower ${i} must trigger through drive contact`);
 for(let k=0;k<240;k++)game.step(1/120);
 assert.equal(f.queue.length,0);assert(game.flowers.filter(x=>x!==f).every(x=>x.queue.length===4));
}
// Corner contact must agree with the rectangle used to stop robot movement.
const corner=setup(),flower=corner.flowers[0];corner.pose={x:flower.x+11,z:flower.z+11.4,theta:0};corner.drive(1,0,0,1/120);assert(flower.released);
for(const css of ['style.css','shot.css'])assert.match(readFileSync('dist/'+css,'utf8'),/-webkit-user-select:none;user-select:none;-webkit-touch-callout:none/);
assert.match(readFileSync('dist/app.js','utf8'),/const visible=!hasDriven/);
console.log('Main-page simulator, immediate loaded-ball firing, all four flower drive contacts, corner contact, label hiding and Safari control CSS checks passed.');
