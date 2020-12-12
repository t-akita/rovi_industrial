#!/usr/bin/env node

const net = require( 'net' );

let host = '127.0.0.1';
let port = 3333;
//let cmdsrc=['X3nd(PE01L)','X0()','X1()','X2()','X3(PE01R)','X0()','X1()','X2()'];
let cmdsrc=['X0()','X3nd(PE01L)','X1()','X2()','X0()','X3(PE01R)','X1()','X2()'];
let cmd=Array.from(cmdsrc);

let client = new net.Socket();

function on_connect(){
  console.log( 'Connected: ' + host + ':' + port );
  if(cmd.length==0){
    cmd=Array.from(cmdsrc);
  }
  let res=cmd.shift();
  if(res.startsWith(cmdsrc[0].substr(0,2))){
    console.log('>>>');
    let block=true;
    process.stdin.on('data',(chunk)=>{
      if(block){
        client.write(res);
        console.log( 'Send:'+res);
      }
      block=false;
    });
  }
  else{
    client.write(res);
    console.log( 'Send:'+res);
  }
}

client.on( 'data', function( data ){
  console.log( 'Recv:'+data);
});

client.on( 'close', function(){
  console.log( 'Disconnected');
  client.connect( port, host, on_connect);
})

client.connect( port, host, on_connect);

