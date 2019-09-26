#!/usr/bin/env node

const parser=require('./rparser');

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
    try{
      let euler=parser.parse(msg);
      let tfs=[];
      for(let i=0;i<euler.length;i++){
        let tf=await who.tflib.fromEuler(euler[i]);
        tfs.push(tf);
      }
      return tfs;
    }
    catch(e){
      console.log('protocol::'+e);
      return [];
    }
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

