// Keep long presses on controls from opening selection or copy menus.
for(const event of ['contextmenu','selectstart'])document.addEventListener(event,e=>{if(e.target instanceof Element&&e.target.closest('button,[data-drive]'))e.preventDefault();});
