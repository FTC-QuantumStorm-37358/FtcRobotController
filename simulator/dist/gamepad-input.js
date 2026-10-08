const clamp=x=>Math.max(-1,Math.min(1,x));
export function deadzone(value,threshold=.18){return Math.abs(value)<=threshold?0:Math.sign(value)*(Math.abs(value)-threshold)/(1-threshold);}
const neutral=()=>({forward:0,strafe:0,turn:0,shoot:false,slower:false,faster:false,connected:false,profile:''});
export function readGamepad(pad){if(!pad?.connected)return neutral();
 // The browser's canonical layout is preferred. F310 raw DirectInput uses X/A/B/Y.
 const raw=pad.mapping!=='standard'&&(/F310|Dual Action|046d|Logitech/i.test(pad.id))&&pad.buttons.length<=12;
 if(pad.mapping!=='standard'&&!raw)return {...neutral(),connected:true,profile:'unsupported'};
 const button=i=>!!(pad.buttons[i]?.pressed||pad.buttons[i]?.value>.5),hat=pad.axes[9];
 let left=button(14),right=button(15),up=button(12),down=button(13);
 if(raw){left=right=up=down=false;if(Number.isFinite(hat)&&hat>=-1&&hat<=1){const direction=Math.round((hat+1)*3.5);up=[0,1,7].includes(direction);right=[1,2,3].includes(direction);down=[3,4,5].includes(direction);left=[5,6,7].includes(direction);}}
 return {forward:clamp(-deadzone(pad.axes[1]??0)+Number(up)-Number(down)),strafe:clamp(deadzone(pad.axes[0]??0)+Number(right)-Number(left)),turn:-deadzone(pad.axes[2]??0)||0,shoot:button(raw?1:0),slower:button(raw?0:2),faster:button(3),connected:true,profile:raw?'F310 DirectInput':'standard'};
}
export class GamepadActions{
 constructor(){this.previous=neutral();this.repeatAt=Infinity;}
 sample(state,now){const shoot=state.shoot&&!this.previous.shoot,dir=Number(state.faster)-Number(state.slower),previousDir=Number(this.previous.faster)-Number(this.previous.slower);let speedStep=0;if(dir){if(dir!==previousDir){speedStep=dir*.01;this.repeatAt=now+400;}else if(now>=this.repeatAt){speedStep=dir*.01;this.repeatAt=now+100;}}else this.repeatAt=Infinity;this.previous=state;return {shoot,speedStep};}
 reset(){this.previous=neutral();this.repeatAt=Infinity;}
}
