#!/usr/bin/env node

module.exports={
//let protocol={
  encode:async function(tf){
    let euler=await this.tflib.toEuler(tf[0]);
    let tarr=[euler[0].toFixed(6),euler[1].toFixed(6),euler[2].toFixed(6)]
    let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
    return "["+tarr.concat(rarr).join(",")+"]";
  },
  decode:async function(msg){
    const who=this;
    let ary=msg.replace(/\).*/g, ']').replace(/.*\(/, '[').replace(/;/, '],[').replace(/E\+/g, 'E').replace(/\+/g, '');
    ary=JSON.parse('['+ary+']');
    let tfs=[];
    for(let i=0;i<ary.length;i++){
      let a=ary[i];
      if(a.length>=6){
        let tf=await who.tflib.fromEuler(a);
        tfs.push(tf);
      }
      else{
        tfs.push(a);
      }
    }
    return tfs;
  },
  tflib:null,
  delim:"\n",
  lf:"\n"
}

/*
const tflib=require('./tflib');
tflib.set('rzyx deg');
protocol.tflib=tflib;

setImmediate(async function(){
  let tf=await protocol.decode("(1,2,3,30,20,AA)");
  console.log("result "+JSON.stringify(tf));
  let d=await protocol.encode(tf);
  console.log("result "+JSON.stringify(d));
});
*/

