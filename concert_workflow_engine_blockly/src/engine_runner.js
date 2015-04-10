#!/usr/bin/env node

var _ = require('lodash'),
  argv = require('minimist')(process.argv.slice(2)),
  winston = require('winston'),
  utils = require('./utils'),
  process_send2 = require('./utils').process_send2,
  Engine = require('./engine');


process.name = 'rocon-workflow-engine'
$pid = process.pid;

global.logger = utils.setup_logger({level: 'info'});
logger.info('pid', $pid);




var _postStatus = function(status, msg){ process.send({action: 'status', status: status, message: msg}); };

var engine_options = JSON.parse(argv.option);

$engine = new Engine(engine_options);

$engine.on('status.*', function(msg){
  var en = this.event.split(/\./)[1];
  msg = msg || '';
  _postStatus(en, msg);

});

$engine.on('ros.**', function(payload){
  var tail = _.tail(this.event.split(/\./));
  var action = tail[0];
  var topic = tail[1];
  process_send2({action: action, topic: topic, payload: payload})
    .then(function(x){
      console.log('XXXX', x);

    });
});



process.on('message', function(data){
  var cmd = data.action || data.cmd;
  switch(cmd){
    case 'run':
      var items = data.items;
      _postStatus('running');
      $engine.runItems(items);
      break;
    case 'stop':
      $engine.clear().then(function(){
        _postStatus('stopped');
      });
      break;
    case 'clear':
      $engine.clear().then(function(){
        _postStatus('stopped');
      });
      break;

  }
});



