// One animation request at a time; a settled scene schedules no more frames.
export class DemandLoop{
 constructor(frame,request=callback=>globalThis.requestAnimationFrame(callback),cancel=id=>globalThis.cancelAnimationFrame(id)){this.frame=frame;this.request=request;this.cancel=cancel;this.pending=null;this.running=false;this.run=now=>{this.pending=null;this.running=true;const again=this.frame(now);this.running=false;if(again)this.wake();};}
 wake(){if(this.pending===null)this.pending=this.request(this.run);}
 pause(){if(this.pending!==null)this.cancel(this.pending);this.pending=null;}
}
