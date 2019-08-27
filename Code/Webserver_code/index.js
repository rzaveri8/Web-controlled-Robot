//Modules
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var XMLHttpRequest = require('xmlhttprequest').XMLHttpRequest;
const net = require('net');
var url ="";
const server = net.createServer((socket) => {
  console.log('\n\n\---client connected---');
  socket.on('data',(data)=>{
      console.log("Msg from client :"+data.toString());
      url = data.toString();

  });
  socket.end("hello car");
}).on('error', (err) => {
  console.log("err:"+err);
  throw err;
});

server.listen({
  host: '0.0.0.0',
  port: 3010,
  exclusive: true
},() => {
  console.log('TCP server started');
});

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
  //Points to stocks.txt
  app.get('/login.html', function(req, res) {
    res.sendFile(__dirname + '/login.html');
  });
});


// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  console.log(url);
  io.emit('url_transmit', url);
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
  socket.on('drive', function(direction){
    if(direction == 0){
      const xhr0 = new XMLHttpRequest();
      const url0 = "http://192.168.1.149/forward";
      xhr0.open("GET", url0);
      xhr0.send();
      console.log("forward");
    }
    else if(direction ==1){
      const xhr1 = new XMLHttpRequest();
      const url1 = "http://192.168.1.149/left";
      xhr1.open("GET", url1);
      xhr1.send();
    }
    else if(direction ==2){
      const xhr0 = new XMLHttpRequest();
      const url2 = "http://192.168.1.149/right";
      xhr0.open("GET", url2);
      xhr0.send();
    }
    else if(direction ==3){
      const xhr3 = new XMLHttpRequest();
      const url3 = "http://192.168.1.149/reverse";
      xhr3.open("GET", url3);
      xhr3.send();
    }
    else if(direction ==4){
      const xhr4 = new XMLHttpRequest();
      const url4 = "http://192.168.1.149/stop";
      xhr4.open("GET", url4);
      xhr4.send();
    };
  });
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});
