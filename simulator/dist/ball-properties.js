// Diameters: BIOBUZZ V1 §9.8. Masses: AndyMark BIOBUZZ Scoring Elements specifications.
export const PROPERTIES={pollen:{diameter:2.8,mass:.055*.45359237},nectar:{diameter:3.6,mass:.091*.45359237}};
export const AIR_DENSITY=1.225,DRAG_COEFFICIENT=.47;
export const ballProperties=type=>PROPERTIES[type==='pollen'?'pollen':'nectar'];
// Approximate sphere drag. The actual perforated balls' coefficient requires measurement.
export function dragK(type){const p=ballProperties(type),radius=p.diameter*.0254/2;return .5*AIR_DENSITY*DRAG_COEFFICIENT*Math.PI*radius*radius/p.mass;}
