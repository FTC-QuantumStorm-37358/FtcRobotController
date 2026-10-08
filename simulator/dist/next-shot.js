import {ballProperties} from './ball-properties.js';
// One immutable queue-head snapshot drives both preview and the subsequent shot.
export function nextShotSettings(item,settings,pose,ratios){if(!item)return null;const type=item.type==='pollen'?'pollen':'nectar';return {...settings,pose:{...pose},type,diameter:ballProperties(type).diameter,transfer:ratios[type],ballId:item.body.id,ballColor:item.type};}

// Firing uses the live queue head directly, without depending on preview drawing.
export function fireNextBall(game,settings,target,ratios,simulate){const next=nextShotSettings(game.nextBall,settings,game.pose,ratios);if(!next){game.message='Empty. Collect a ball with the front intake.';return false;}const shot=simulate(next,target);shot.ballId=next.ballId;return game.shoot(shot);}

export function previewNextBall(game,settings,target,ratios,simulate){const next=nextShotSettings(game.nextBall,settings,game.pose,ratios);if(!next)return null;const shot=simulate(next,target);shot.ballId=next.ballId;shot.ballColor=next.ballColor;return {settings:next,shot};}
export const shotColor=type=>type==='pollen'?'#f5d650':type==='red'?'#ef6961':'#629cef';
