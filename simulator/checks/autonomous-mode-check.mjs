import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {AUTONOMOUS_MODES,autonomousMode,autonomousStart,placeAutonomousRobot} from '../dist/autonomous-modes.js';
import {JavaAutonomousClient} from '../dist/autonomous.js';
import {fencePose,INCH} from '../dist/ballistics.js';
import {PracticeScore} from '../dist/practice-score.js';
import {Penalties} from '../dist/penalties.js';
const html=readFileSync('dist/index.html','utf8');assert.equal(html,readFileSync('dist/shot.html','utf8'));
assert(html.includes('id="score-note"'),'Alliance scoring help must have a stable DOM identifier');
assert.equal(AUTONOMOUS_MODES.length,8);assert.equal(new Set(AUTONOMOUS_MODES.map(m=>m.id)).size,8);
for(const mode of AUTONOMOUS_MODES){
 assert(html.includes(`<option value="${mode.id}">${mode.label}</option>`));assert.equal(autonomousMode(mode.id),mode);
 const source=readFileSync(new URL(`../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/quantumstorm/Auto/${mode.file}`,import.meta.url),'utf8');assert(source.includes(`name = "${mode.label}"`)||source.includes(`name="${mode.label}"`));
 const game={pose:null,rules:new Penalties(),syncRobot(){this.synced=true;},preparePartnerTilt(a){this.partner=a;}};placeAutonomousRobot(game,mode);assert.deepEqual(game.pose,autonomousStart(mode));assert.equal(game.partner,mode.teamRole===2?mode.alliance:undefined);assert.equal(game.rules.alliance,mode.alliance);assert(game.synced);
 let socket;const packets=[];const driver=new JavaAutonomousClient(game,{url:'ws://test/robot',socketFactory:()=>socket={send:text=>packets.push(JSON.parse(text)),close(){}}});
 driver.sensors.read=()=>({pose:{x:game.pose.x,y:-game.pose.z,heading:0},sampledAt:0,odometryValid:true,cameraConnected:false,ballCount:4,vision:{valid:false,pipeline:0,capturedAt:0,targets:[]}});
 driver.start(mode.alliance,.395,mode);socket.onmessage({data:JSON.stringify({type:'ready',protocol:1,engine:'java',tickSeconds:.02,nearTags:[],farTags:[]})});
 assert.equal(packets[0].alliance,mode.alliance);assert.equal(packets[0].cameraFree,true);assert.equal(packets[0].singleTilt,mode.singleTilt);assert.equal(packets[0].teamRole,mode.teamRole??0);assert.equal(packets[0].seq,0);driver.stop();assert.equal(driver.pending,false);
}
assert.throws(()=>autonomousMode('unknown'));
const red={},blue={},rules=new Penalties(1),game={pose:{x:59,z:30,theta:0},rules,
 field:{userData:{loadingZones:[{x:-66.5,z:-36,hx:5.5,hz:11.5},{x:66.5,z:36,hx:5.5,hz:11.5}]}},
 hives:[{tilts:0,cells:[red],counts:[{pollen:0,nectar:0}]},{tilts:2,cells:[blue],counts:[{pollen:1,nectar:1}]}],raisedCell:i=>i===0?red:blue,flowers:[],
 balls:[{held:false,storedInFlower:false,type:'pollen',r:1.4*INCH,body:{position:{x:60.5*INCH,y:1.4*INCH,z:-70.8*INCH}}}]};
const score=new PracticeScore(1);score.setPeriod('teleop',game);const result=score.read(game);assert.equal(result.tilts,2);assert.equal(result.rows.find(r=>r.name==='Blue garden').points,1);assert.equal(result.rows.find(r=>r.name==='AUTO park').points,5);
rules.collected({type:'red',body:{id:1},pickups:1});rules.collected({type:'blue',body:{id:2},pickups:1});assert.equal(rules.events.filter(e=>e.id==='G408').length,1);assert.match(rules.events[0].detail,/Blue robot collected red/);
console.log('Eight mode/file mappings, alliance resets, camera-free/single-tilt packets, blue scoring and opponent penalties passed.');
