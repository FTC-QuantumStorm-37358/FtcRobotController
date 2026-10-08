import {INCH} from './ballistics.js';
// Manual V1 §§10.5.1–10.5.5. Live practice estimates; end-period results
// are sampled when switching AUTO → TELEOP, not inferred from actuator timers.
export function chassisCorners(p){const c=Math.cos(p.theta),s=Math.sin(p.theta);return [-8.75,8.75].flatMap(x=>[-9.05,8.055].map(z=>({x:p.x+c*x+s*z,z:p.z-s*x+c*z})));}
export function leave(p){return chassisCorners(p).every(v=>Math.abs(v.x)<71.9&&Math.abs(v.z)<71.9);}
export function park(p,zone){
 const a=chassisCorners(p),b=[-zone.hx,zone.hx].flatMap(x=>[-zone.hz,zone.hz].map(z=>({x:zone.x+x,z:zone.z+z}))),c=Math.cos(p.theta),s=Math.sin(p.theta);
 return [{x:1,z:0},{x:0,z:1},{x:c,z:-s},{x:s,z:c}].every(n=>{const aa=a.map(v=>v.x*n.x+v.z*n.z),bb=b.map(v=>v.x*n.x+v.z*n.z);return Math.min(...aa)<Math.max(...bb)&&Math.min(...bb)<Math.max(...aa);});
}
export class PracticeScore{
 constructor(){this.reset();}
 reset(){this.period='auto';this.auto=null;}
 setPeriod(period,game){if(period===this.period)return;if(period==='teleop')this.auto=this.measure(game);else this.auto=null;this.period=period;}
 measure(game){return {tilts:game.hives[0].tilts,leave:leave(game.pose)?3:0,park:park(game.pose,game.field.userData.loadingZones[0])?5:0};}
 read(game){
  const measured=this.measure(game),auto=this.auto??measured,tele=this.period==='teleop',tilts=game.hives[0].tilts,rows=[];
  rows.push({name:'Hive tilts',detail:`${tilts} completed × 20`,points:tilts*20});
  rows.push({name:'AUTO leave',detail:(auto.leave?'Clear of perimeter':'Still touching perimeter')+(tele?' · saved at AUTO end':' · live'),points:auto.leave});
  rows.push({name:'AUTO park',detail:(auto.park?'In red loading zone':'Outside red loading zone')+(tele?' · saved at AUTO end':' · live'),points:auto.park});
  rows.push({name:'TELEOP park',detail:tele?(measured.park?'In red loading zone':'Outside red loading zone'):'End of TELEOP only',points:tele?measured.park:0});
  const hive=game.hives[0],raised=game.raisedCell(0),counts=hive.counts[hive.cells.indexOf(raised)],cell=counts.pollen+counts.nectar;
  let garden=0;const flowers=game.flowers.map(f=>({f,items:[]}));
  for(const ball of game.balls){
   if(ball.held)continue;const p={x:ball.body.position.x/INCH,y:ball.body.position.y/INCH,z:ball.body.position.z/INCH},r=ball.r/INCH;
   // At least partial garden overlap; balls in the air above the garden qualify
   // provisionally because this zone is infinitely tall (final assessment waits for rest).
   if(!ball.storedInFlower&&p.x+r>=-72&&p.x-r<=-49&&p.z+r>=69.8&&p.z-r<=71.8)garden++;
   for(const flower of flowers){const f=flower.f;if(Math.hypot(p.x-f.x,p.z-f.z)<2.22+r&&p.y+r>4.4&&p.y-r<21.5)flower.items.push({type:ball.type,y:p.y});}
  }
  let owned=0,bottom=0;
  for(const {items} of flowers){const nectar=items.filter(v=>v.type!=='pollen').sort((a,b)=>a.y-b.y);if(nectar[0]?.type==='red')bottom++;if(nectar.at(-1)?.type==='red')owned+=items.length;}
  const when=tele?' · provisional until match end':' · end of TELEOP only';
  rows.push({name:'Raised-cell balls',detail:`${cell} balls × 2${when}`,points:tele?cell*2:0});
  rows.push({name:'Owned-flower balls',detail:`${owned} balls × 2${when}`,points:tele?owned*2:0});
  rows.push({name:'Bottom nectar bonus',detail:`${bottom} flowers × 5${when}`,points:tele?bottom*5:0});
  rows.push({name:'Red garden',detail:`${garden} balls × 1${when}`,points:tele?garden:0});
  const earned=rows.reduce((n,r)=>n+r.points,0);return {total:game.rules?.disqualified?0:earned,earned,opponentPenaltyPoints:game.rules?.points??0,disqualified:game.rules?.disqualified??false,rows,tilts,autoTilts:auto.tilts,teleTilts:tele?tilts-auto.tilts:0};
 }
}
