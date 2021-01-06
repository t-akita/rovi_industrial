#!/usr/bin/env node

const net = require( 'net' );

let host = '127.0.0.1';
let port = 3333;
let cmdsrc=['X3(PE01L)','X0()','X1()','X2()','X3(PE01R)','X0()','X1()','X2()'];
//let cmdsrc=['X0()','X3nd(PE01L)','X1()','X2()','X0()','X3(PE01R)','X1()','X2()'];
let cmd=Array.from(cmdsrc);
let client = new net.Socket();
let error=0;

function on_connect(){
  console.log( 'Connected: ' + host + ':' + port );
  if(cmd.length==0){
    cmd=Array.from(cmdsrc);
  }
  let res=cmd.shift();
  client.write(res);
}

client.on( 'data', function( data ){
  console.log( 'Recv:'+data);
  let msg=data.toString();
  if(msg.startsWith('NG')){
    let cod=Number(msg.substr(3));
    console.log("error code:"+cod);
    if(cod!=921) error++;
  }
});

client.on( 'close', function(){
  console.log( 'Disconnected');
  if(error==0) client.connect( port, host, on_connect);
})

client.connect( port, host, on_connect);

