import {INCH} from './ballistics.js';
// Manual V1 Table 10-4 and §§11.1–11.4. Cards are match-result assessments.
const W={label:'Verbal warning',warning:true},M={label:'Major foul',major:1},MY={label:'Strategic: major foul + yellow card',major:1,yellow:true},Y={label:'Strategic: yellow card',yellow:true},R={label:'Red card',red:true},D={label:'Disable robot',disabled:true},H={label:'Hold match until corrected',hold:true};
export const RULES={};
function rule(id,title,options){RULES[id]={id,title,options};}
for(const [id,title] of [['G101','Human entered field'],['G102','Human misused arena']])rule(id,title,[W]);
rule('G201','Unsporting conduct',[W,{label:'Subsequent event violation: yellow card',yellow:true}]);
rule('G202','Competition integrity',[W,{...Y,label:'Subsequent event violation: yellow card'},{...R,label:'Subsequent event violation: red card'}]);
rule('G203','Missing drive team',[{label:'Disqualified',dq:true}]);
rule('G204','Forcing opponent violation',[M,{label:'Repeated: major foul + yellow card',major:1,yellow:true}]);
rule('G205','Egregious behavior',[{...Y,label:'Yellow card'},R]);
rule('G301','Match delay',[W,{label:'Subsequent phase violation: major foul',major:1},D]);
rule('G302','Prohibited field equipment',[H,W,{label:'Subsequent event violation: yellow card',yellow:true}]);
rule('G303','Robot not match ready',[H,D,{...R,label:'Uninspected robot participated: red card'}]);
rule('G304','Illegal starting setup',[H,D]);rule('G305','OpMode not initialized',[H,D]);
for(const [id,title] of [['G401','Driver input during AUTO'],['G403','Powered movement during transition'],['G404','Powered movement after TELEOP'],['G406','Arena damage or mess'],['G407','Control more than four balls'],['G417','Manipulating hive motion'],['G418','Illegal flower entry or removal'],['G423','Coach or other team on controls'],['G425','Human field contact']])rule(id,title,[W,MY,...(id==='G406'?[{label:'Damage likely: disable robot',disabled:true}]:[])]);
rule('G402','AUTO opponent interference',[{...M,label:'Major foul per match'},MY]);
rule('G405','Deliberate ball ejection',[{...M,label:'Major foul per ball'}]);
rule('G408','Control opponent nectar',[W,Y]);rule('G409','Catch fresh hive spill',[W,Y]);
rule('G410','Nectar in flower before last 60 seconds',[{...M,label:'Major foul per nectar'}]);
rule('G411','Strategic ball hoarding',[MY]);
rule('G412','Dangerous robot',[{label:'Disable + warning',disabled:true,warning:true},{label:'Subsequent event violation: disable + warning + yellow',disabled:true,warning:true,yellow:true}]);
rule('G413','Ignoring referee stop',[W,{...R,label:'Strategic: red card'}]);rule('G414','Unidentifiable robot',[W,Y]);rule('G415','Grabbing or hanging from arena',[W,Y,{label:'Damage likely: disable robot',disabled:true}]);
rule('G416','Illegal expansion or detached parts',[W,{...M,label:'Strategic: major foul per instance'}]);
for(const [id,title] of [['G419','Damage opponent robot'],['G420','Tip or entangle opponent']])rule(id,title,[W,{...MY,label:'Strategic: major + yellow per instance'},{label:id==='G420'?'Strategic and continuous / cannot drive: major + red':'Strategic and cannot drive: major + red',major:1,red:true}]);
rule('G421','Pin over three seconds',[{...M,label:'Major foul; add one per further 3 seconds'}]);
for(const [id,title] of [['G422','Human outside alliance area'],['G424','Coach touched ball']])rule(id,title,[W,{label:'Strategic: minor foul per instance',minor:1}]);
for(const [id,title] of [['G426','Human introduced nectar early'],['G427','Illegal human nectar introduction'],['G428','Human removed field ball']])rule(id,title,[{label:'Minor foul per ball',minor:1}]);
rule('T405','Driving during field measurement',[W,{label:'Subsequent event violation: yellow card',yellow:true}]);
export class Penalties{
 constructor(){this.reset();}
 reset(){this.events=[];this.keys=new Set();this.matchRules=new Set();this.yellow=0;this.red=false;this.disabled=false;this.held=false;this.timed=false;this.started=0;this.phase='auto';this.teleopElapsed=0;this.time=0;this.active=new Set();}
 start(now){this.reset();this.timed=true;this.started=now;}
 updateClock(now){if(!this.timed)return;this.time=Math.max(0,(now-this.started)/1000);this.phase=this.time<30?'auto':this.time<38?'transition':this.time<158?'teleop':'ended';this.teleopElapsed=Math.max(0,this.time-38);}
 get lastMinute(){return this.phase==='teleop'&&this.teleopElapsed>=60;}
 get points(){return this.events.reduce((n,e)=>n+e.points,0);}
 get disqualified(){return this.red||this.events.some(e=>e.dq);}
 assess(id,index=0,{key=null,count=1,detail='',forced=false,source='Referee'}={}){
  if(forced||key&&this.keys.has(key))return false;
  const r=RULES[id],a=r?.options[index];if(!a)throw new Error('Unknown rule assessment');
  count=Math.max(1,Math.min(100,Math.floor(Number(count)||1)));
  if(key)this.keys.add(key);
  // "per MATCH" assessments are capped for this rule and assessment, not per ball.
  const perMatch=['G401','G402','G403','G404','G406','G407','G408','G409','G411','G412','G415','G417','G418','G423','G425'].includes(id)&&(a.major||a.yellow);
  const capKey=id+':'+index;if(perMatch&&this.matchRules.has(capKey))return false;if(perMatch)this.matchRules.add(capKey);
  const yellow=!!a.yellow&&!(id==='G204'&&this.keys.has('G204:yellow'));if(yellow){this.yellow+=['G419','G420'].includes(id)?count:1;if(id==='G204')this.keys.add('G204:yellow');}if(a.red||this.yellow>=2)this.red=true;
  this.disabled||=!!a.disabled;this.held||=!!a.hold;
  this.events.push({id,title:r.title,label:a.label,points:((a.minor||0)*5+(a.major||0)*20)*(perMatch?1:count),count:perMatch?1:count,warning:!!a.warning,disabled:!!a.disabled,hold:!!a.hold,yellow,red:!!a.red||yellow&&this.yellow>=2,dq:!!a.dq,detail,source,time:this.time});return true;
 }
 episode(id,on,detail=''){if(!on){this.active.delete(id);return;}if(this.active.has(id))return;this.active.add(id);this.assess(id,0,{detail,source:'Automatic'});}
 manualInput(autonomous=false){if((this.timed||autonomous)&&this.phase==='auto')this.episode('G401',true,'Driver input during AUTO; stop and safety remain allowed.');}
 powered(on){if(!on){this.active.delete('G403');this.active.delete('G404');this.active.delete('G401');return;}if(this.timed&&['transition','ended'].includes(this.phase))this.episode(this.phase==='transition'?'G403':'G404',true);}
 collected(item){this.active.delete('G407');if(item.type==='blue')this.assess('G408',0,{key:`G408:${item.body.id}:${item.pickups}`,detail:'Red robot collected blue nectar.',source:'Automatic'});if(item.freshSpill)this.assess('G409',0,{key:`G409:${item.body.id}:${item.spills}`,source:'Automatic'});}
 sample(game){
  this.episode('G407',game.inventory.length>4);
  for(const b of game.balls){if(b.held||b.storedInFlower||!b.robotHandled)continue;
   const p=b.body.position,r=b.r,x=p.x/INCH,y=p.y/INCH,z=p.z/INCH,ri=r/INCH;
   const flower=game.flowers.find(f=>Math.hypot(x-f.x,z-f.z)<2.22+ri&&y+ri>4.4&&y-ri<21.5);
   if(flower&&b.penaltyFlower!==flower.index){b.penaltyFlower=flower.index;
    if(b.type!=='pollen'&&this.phase!=='ended'&&!this.lastMinute)this.assess('G410',0,{key:`G410:${b.body.id}`,detail:'Nectar entered flower before the final TELEOP minute.',source:'Automatic'});
    const previous=b.previousRulePosition;if(previous&&previous.y-ri<21.5)this.assess('G418',0,{key:`G418:${b.body.id}:${flower.index}`,detail:'Ball entered modeled flower volume from side or bottom.',source:'Automatic'});
   }else if(!flower)b.penaltyFlower=null;
   b.previousRulePosition={x,y,z};
  }
 }
}
