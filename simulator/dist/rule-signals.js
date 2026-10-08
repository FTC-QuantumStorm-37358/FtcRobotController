// Local referee presentation only: no robot control or autonomous decisions.
export function ruleAnnouncement(e){
 const parts=[];
 if(e.warning)parts.push('Verbal warning');
 if(e.points)parts.push(e.label.toLowerCase().includes('minor')?'Minor foul':'Major foul');
 if(e.red)parts.push('Red card. Disqualified. Red score is zero');
 else if(e.dq)parts.push('Disqualified. Red score is zero');
 else if(e.yellow)parts.push('Yellow card');
 if(e.disabled)parts.push('Robot disabled. Stop all powered actions');
 if(e.hold)parts.push('Match held. Correct the starting issue');
 if(!parts.length&&/disable/i.test(e.label))parts.push('Robot disabled. Stop all powered actions');
 if(!parts.length&&/hold match/i.test(e.label))parts.push('Match held. Correct the starting issue');
 return `${parts.join('. ')}. ${e.id}. ${e.title}.${e.points?` Blue receives ${e.points} penalty points.`:''}${e.detail?' '+e.detail:''}`;
}
export class RuleSignals{
 constructor({show,clear}){this.show=show;this.clear=clear;this.events=null;this.seen=0;}
 consume(events){
  if(events!==this.events){this.clear();this.events=events;this.seen=0;}
  for(;this.seen<events.length;this.seen++){const e=events[this.seen];this.show(ruleAnnouncement(e),e);}
 }
}
