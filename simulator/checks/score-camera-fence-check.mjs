import assert from 'node:assert/strict';
import {readFileSync,existsSync} from 'node:fs';
import * as T from '../dist/vendor/three.module.js';
import {followFrame} from '../dist/follow-camera.js';
import {PracticeScore,leave,park} from '../dist/practice-score.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {GamePhysics}=await import('../dist/game-physics.js');const {buildField,buildRobot}=await import('../dist/model.js');const {INCH}=await import('../dist/ballistics.js');
const setup=()=>{const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);return new GamePhysics(field,scene,robot);};
const game=setup(),score=new PracticeScore();assert.equal(score.read(game).total,0,'End-match preloads must not earn points in AUTO');assert(!leave(game.pose));
game.pose={x:-56,z:-36,theta:Math.PI};assert(leave(game.pose));assert(park(game.pose,game.field.userData.loadingZones[0]));
game.hives[0].tilts=2;assert.equal(score.read(game).total,48,'Two completed tips + AUTO leave + AUTO park');
score.setPeriod('teleop',game);const tele=score.read(game);assert.equal(tele.rows.find(r=>r.name==='TELEOP park').points,5);assert.equal(tele.rows.find(r=>r.name==='Raised-cell balls').points,6);assert(tele.total>48);
game.pose={x:-45,z:-40,theta:0};game.hives[0].tilts=3;const changed=score.read(game);assert.equal(changed.rows.find(r=>r.name==='AUTO park').points,5,'AUTO results must stay frozen');assert.equal(changed.rows.find(r=>r.name==='TELEOP park').points,0);assert.equal(changed.teleTilts,1);
// Ownership follows the top-most nectar; bottom bonus follows the lowest.
const flower=game.flowers[0];game.createBall('red',[flower.x,8,flower.z]);game.createBall('blue',[flower.x,15,flower.z]);let row=score.read(game).rows;assert.equal(row.find(r=>r.name==='Bottom nectar bonus').points,5);assert.equal(row.find(r=>r.name==='Owned-flower balls').points,0);
game.createBall('red',[flower.x,19,flower.z]);row=score.read(game).rows;assert(row.find(r=>r.name==='Owned-flower balls').points>=6);
score.reset();assert.equal(score.period,'auto');assert.equal(score.read(game).rows.find(r=>r.name==='Red garden').points,0);
const a=followFrame({x:10,z:20,theta:0}),b=followFrame({x:10,z:20,theta:Math.PI/2});assert.equal(a.position.z,110);assert.equal(a.target.z,-10);assert(Math.abs(b.position.x-100)<1e-8);assert(Math.abs(b.target.x+20)<1e-8);
const translated=followFrame({x:25,z:12,theta:0});assert.equal(translated.position.x-a.position.x,15);assert.equal(translated.target.z-a.target.z,-8);
assert(followFrame({x:0,z:0,theta:0},.5).position.z>a.position.z);
// Fast/high shots and corner trajectories must remain within the full sphere
// boundary, even when they would pass above the physical twelve-inch fence.
const fence=setup();for(const type of ['pollen','red'])for(const height of [2,70])for(const side of [-1,1]){
 const ball=fence.createBall(type,[side*69,height,side*69]);ball.body.velocity.set(side*25,2,side*25);fence.step(1/30);
 assert(Math.abs(ball.body.position.x)+ball.r<=72*INCH);assert(Math.abs(ball.body.position.z)+ball.r<=72*INCH);assert(ball.body.velocity.x*side<=0);assert(ball.body.velocity.z*side<=0);
}
const pushed=fence.createBall('pollen',[72,1.41,0]);fence.containBalls();assert(pushed.body.position.x+pushed.r<=72*INCH);
const html=readFileSync('dist/index.html','utf8'),js=readFileSync('dist/shot.js','utf8');assert.equal(html,readFileSync('dist/shot.html','utf8'));assert(!existsSync('dist/explore.html'));
for(const id of ['alliance','fence','shoot-touch','autonomous-touch','autonomous-overlay','pose-note','curve-ball','red-load','blue-load','flower-load'])assert(!html.includes(`id="${id}"`),`${id} should be removed`);
for(const id of ['score-total','score-period','score-breakdown','follow','aim','suggest','reset'])assert(html.includes(`id="${id}"`));
assert(!html.includes('Field explorer'));assert(!readFileSync('dist/model.js','utf8').includes("plate=sign(structure,'BIOBUZZ'"));assert(html.indexOf('id="reset"')<html.indexOf('Robot inventory'));assert(html.indexOf('id="aim"')<html.indexOf('id="suggest"'));
const ids=new Set([...html.matchAll(/id="([^"]+)"/g)].map(m=>m[1]));for(const [,id] of js.matchAll(/\$\('([^']+)'\)/g))assert(ids.has(id),`Missing element ${id}`);
console.log('Phase-aware scores, final-period ball points, flower ownership, camera heading/translation, high-speed fence containment and simplified UI contracts passed.');
