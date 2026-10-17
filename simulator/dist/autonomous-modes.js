import {fencePose} from './ballistics.js';

// OpMode metadata and initial placement only; all routine decisions remain in Java.
export const AUTONOMOUS_MODES=Object.freeze([
 {id:'red-one',label:'Red Team One Auto',alliance:0,teamRole:1,singleTilt:false,file:'RedTeamOneAutonomous.java'},
 {id:'red-two',label:'Red Team Two Auto',alliance:0,teamRole:2,singleTilt:false,file:'RedTeamTwoAutonomous.java'},
 {id:'blue-one',label:'Blue Team One Auto',alliance:1,teamRole:1,singleTilt:false,file:'BlueTeamOneAutonomous.java'},
 {id:'blue-two',label:'Blue Team Two Auto',alliance:1,teamRole:2,singleTilt:false,file:'BlueTeamTwoAutonomous.java'},
 {id:'red-one-single-tilt',label:'Red Team One Single Tilt',alliance:0,singleTilt:true,file:'RedTeamOneSingleTiltAutonomous.java'},
 {id:'red-two-single-tilt',label:'Red Team Two Single Tilt',alliance:0,singleTilt:true,file:'RedTeamTwoSingleTiltAutonomous.java'},
 {id:'blue-one-single-tilt',label:'Blue Team One Single Tilt',alliance:1,singleTilt:true,file:'BlueTeamOneSingleTiltAutonomous.java'},
 {id:'blue-two-single-tilt',label:'Blue Team Two Single Tilt',alliance:1,singleTilt:true,file:'BlueTeamTwoSingleTiltAutonomous.java'}
].map(mode=>Object.freeze({...mode,cameraFree:true})));
export function autonomousMode(id){const mode=AUTONOMOUS_MODES.find(mode=>mode.id===id);if(!mode)throw new Error('Unknown autonomous mode');return mode;}
// Halfway along the side wall between Loading Zone (Z -36) and Flower (Z +24).
export function autonomousStart(mode){return mode.teamRole===2?(mode.alliance===0?{x:-62.9,z:-6,theta:Math.PI}:{x:62.9,z:6,theta:0}):fencePose(mode.alliance);}
export function placeAutonomousRobot(game,mode){game.pose=autonomousStart(mode);game.rules.alliance=mode.alliance;game.syncRobot();if(mode.teamRole===2)game.preparePartnerTilt(mode.alliance);}
