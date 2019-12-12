#!/usr/bin/env node

const net=require('net');

let Config={
  port:8888
};

let theta=[0,0,0,0,0,0];
let vel=[0.1,0.1,0.1,0.1,0.1,0.1];

setImmediate(async function(){
  const server = net.createServer(function(conn){
    conn.setTimeout(0);
    const rep=setInterval(function(){
      for(let i=0;i<theta.length;i++){
        theta[i]+=vel[i];
        if(Math.abs(theta[i])>1.5) vel[i]=-vel[i];
      }
      conn.write(theta.toString()+'\n');
    },300);
    conn.on('data',async function(data){
      console.log('connected');
    });
    conn.on('close', function(){
      clearTimeout(rep);
      console.log('closed');
    });
    conn.on('timeout',function(){
      console.log('timeout');
      conn.destroy();
    });
    conn.on('error', function(err){
      console.log('error');
    });
  }).listen(Config.port);
});
