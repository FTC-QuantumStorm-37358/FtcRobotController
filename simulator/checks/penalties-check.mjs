import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {Penalties,RULES} from '../dist/penalties.js';
import {PracticeScore} from '../dist/practice-score.js';
import * as T from '../dist/vendor/three.module.js';
import {buildField,buildRobot} from '../dist/model.js';
import {GamePhysics} from '../dist/game-physics.js';
import {INCH} from '../dist/ballistics.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const scene=new T.Scene(),field=buildField(),robot=buildRobot();scene.add(field,robot);const game=new GamePhysics(field,scene,robot),r=game.rules;
assert.equal(Object.keys(RULES).length,41);
// Real measured pickup: warning, no invented foul points or automatic intent.
const blue=game.balls.find(b=>b.type==='blue'&&b.startZone==='loading');game.inventory.pop().held=false;assert(game.collect(blue));assert.equal(r.events.at(-1).id,'G408');assert.equal(r.points,0);assert.equal(r.yellow,0);
r.assess('G408',1);r.assess('G408',1);assert.equal(r.yellow,1); // per MATCH
r.assess('G409',1);assert(r.red);assert.equal(new PracticeScore().read(game).total,0);
game.reset();assert.equal(game.rules.events.length,0);assert(!game.rules.disabled);
// Original field nectar is not charged; robot-handled nectar entering early is.
const nectar=game.balls.find(b=>b.type==='red'&&b.startZone==='loading'),f=game.flowers[0];nectar.body.position.set(f.x*INCH,10*INCH,f.z*INCH);r.sample(game);assert.equal(r.points,0);nectar.robotHandled=true;r.sample(game);r.sample(game);assert.equal(r.points,20);assert.equal(r.events[0].id,'G410');assert.equal(new PracticeScore().read(game).total,0);assert.equal(new PracticeScore().read(game).opponentPenaltyPoints,20);
r.reset();nectar.penaltyFlower=null;nectar.previousRulePosition=null;r.start(0);r.updateClock(97999);assert(!r.lastMinute);r.updateClock(98000);assert(r.lastMinute);r.sample(game);assert.equal(r.points,0);
// Clock continues independently of physics; exact period boundaries.
r.updateClock(29999);assert.equal(r.phase,'auto');r.manualInput();r.manualInput();assert.equal(r.events.length,1);r.updateClock(30000);assert.equal(r.phase,'transition');r.powered(true);r.powered(true);assert.equal(r.events.length,2);r.updateClock(38000);assert.equal(r.phase,'teleop');r.updateClock(158000);assert.equal(r.phase,'ended');r.powered(true);assert.equal(r.events.at(-1).id,'G404');
r.reset();r.assess('G405',0,{count:3});assert.equal(r.points,60);r.assess('G426',0,{count:2});assert.equal(r.points,70);r.assess('G410',0,{forced:true});assert.equal(r.points,70);r.assess('G421',0,{count:6});assert.equal(r.points,190);
r.assess('G412',0);const pose={...game.pose},shots=game.shots;assert.equal(game.drive(1,0,0,.02),false);assert.deepEqual(game.pose,pose);assert.equal(game.shoot({speed:10}),false);assert.equal(game.shots,shots);
// Capacity guard and incidental pushes do not produce CONTROL penalties.
game.reset();r.sample(game);assert(!r.events.some(e=>e.id==='G407'));assert.equal(game.collect(game.balls.find(b=>!b.held&&!b.storedInFlower)),false);r.sample(game);assert.equal(r.events.length,0);
const page=readFileSync('dist/index.html','utf8');assert.equal(page,readFileSync('dist/shot.html','utf8'));for(const id of ['penalty-log','penalty-rule','penalty-assessment','start-match','penalty-points'])assert(page.includes(`id="${id}"`));
console.log('Manual penalty values, actual opponent-nectar pickup, early/late flower entry, phase boundaries, warning deduplication, per-match caps, forced exemptions, card escalation, DQ score, disabling and reset passed.');
