var spawn = require('child_process').spawn,
  _ = require('lodash'),
  Promise = require('bluebird'),
  Engine = require('./engine'),
  ResourceManager = require('./resource_manager'),
  util = require('util'),
  Requester = require('./requester').Requester,
  Resource = require('./requester').Resource,
  EventEmitter2 = require('eventemitter2').EventEmitter2;



var EngineManager = function(io, options){
  var that = this;
  EventEmitter2.call(this, {wildcard: true});

  this.io = io;
  this.engine_processes = {};

  var ros = this.ros = new Ros(options);
  ros.on('status.**', function(){});

  this.resource_pool_status = null;
  ros.once('status.ready', function(){

    ros.subscribe('/concert/scheduler/resource_pool', 'scheduler_msgs/KnownResources', function(payload){
      logger.debug("POOL", payload);
      that.resource_pool_status = payload;

      that.io.of('/engine/client')
        .emit('data', {event: 'resource_pool', payload: payload});
    });


    ros.subscribe('enable_workflows', 'concert_workflow_engine_msgs/EnableWorkflows', _.bind(that.enableWorkflows, that));

  });

  this.resource_manager = new ResourceManager(ros);
  this.resource_manager.onAny(function(){
    that.broadcastResourcesInfo();
  });


  this.options = options;

  this.onAny(function(payload){
    io.emit('data', {event: this.event, payload: payload});
  });


  this.io.of('/prezi').on('connection', function(socket){
    logger.info('socket.io(prezi) connected id:%s', socket.id);
    
  });



  this.io.of('/engine/client').on('connection', function(socket){
    logger.info('socket.io client connected id:%s', socket.id);
    that._bindClientSocketHandlers(socket);
  });
  this.io.of('/blockly').on('connection', function(socket){
    logger.info('socket.io(blockly) connected id:%s', socket.id);
    var mgr = that;

    socket.on('*', function(e){
      var cmd = e.data[0];

      switch(cmd){
        case 'start':
          var pid = mgr.startEngine();
          var payload = e.data[1];
          if(payload && payload.items && payload.items.length){
            mgr.run(pid, payload.items);
          }
          break;
      }

    });

  });

};
util.inherits(EngineManager, EventEmitter2);



EngineManager.prototype.enableWorkflows = function(options){
  var mgr = this;
  var payload = options;
  if(payload.enable){

    if(_.find(this.engine_processes, {name: payload.service_name})){
      logger.error(payload.service_name + " is already running");
    }else{
      var pid = this.startEngine({name: payload.service_name});
      var workflows = payload.workflows;
      mgr.run(pid, workflows);

    }




    
  }else{
    var x = _.find(this.engine_processes, {name: payload.service_name})
    this.killEngine(x.process.pid)
  }


};

EngineManager.prototype._bindClientSocketHandlers = function(socket){
  var mgr = this;

  socket.on('*', function(e){
    // e.nsp
    // e.data
    var cmd = e.data[0];

    switch(cmd){
      case 'get_processes':
        mgr.broadcastEnginesInfo();
        break;

      case 'get_resources':
        mgr.broadcastResourcesInfo();
        break;

      case 'start':
        var pid = mgr.startEngine();
        var payload = e.data[1];
        if(payload && payload.items && payload.items.length){
          mgr.run(pid, payload.items);
        }
        break;

      case 'run':
        var payload = e.data[1];
        mgr.run(payload.pid, payload.items);
        break;

      case 'kill':
        var payload = e.data[1];
        mgr.killEngine(payload.pid);
        break;

      case 'release_resource':
        var payload = e.data[1];
        mgr.resource_manager.release(payload.requester);
        break;
    }


  });


};





EngineManager.prototype.startEngine = function(extras){
  var engine_opts = this.options;
  extras = _.defaults(extras || {}, {});

  logger.info('engine options', engine_opts);

  var child = spawn('node', [__dirname + '/engine_runner.js', '--option', JSON.stringify(engine_opts)], {stdio: ['pipe', 'pipe', 'pipe', 'ipc']})
  global.childEngine = child;

  logger.info("engine spawn pid :", child.pid);

  child.stdout.on('data', function(data){
    logger.info("engine-" + child.pid, data.toString().trim());
  });
  child.stderr.on('data', function(data){
    logger.info("engine-" + child.pid, data.toString().trim());
  });


  var data = {process: child, ee: new EventEmitter2({wildcard: true})};
  if(extras.name){
    data.name = extras.name;
  }else{
    data.name = "Engine#" + child.pid;
  }
  this.engine_processes[child.pid] = data;
  this._bindEvents(child);


  return child.pid;
}

EngineManager.prototype.broadcastMessage = function(msg){
  return _(this.engine_processes)
    .each(function(child, pid){
      var proc = child.process;
      proc.send(msg);
    }).value();



};


EngineManager.prototype.callOnReady = function(pid, cb) {
  var c = this.engine_processes[pid];
  if(c && c.status && c.status == 'ready'){
    cb.call(this);
  }else{
    this.once(['child', pid, 'ready'].join('.'), cb.bind(this));
  }
  
}

EngineManager.prototype.run = function(pid, workflows){
  var that = this;

  this.callOnReady(pid, function(){
    var child = that.engine_processes[pid];
    var proc = child.process;
    var items_to_load = _.map(workflows, 'data');
    if(_.isArray(items_to_load) && _.isString(items_to_load[0])){
      items_to_load = _.map(items_to_load, function(i){ return JSON.parse(i); });
    }

    proc.send({action: 'run', items: items_to_load});
    child.running_items = items_to_load;


    // if(_.isString(workflows[0])){
      // var items = Settings.getItems(function(e, items){
        // var items_to_load = _(items)
          // .filter(function(i) { return _.contains(workflows, i.title); })
          // .sortBy(function(i) { return _.indexOf(workflows, i.title); })
          // .value();
        // var child = that.engine_processes[pid];
        // var proc = child.process;
        // proc.send({action: 'run', items: items_to_load});
        // child.running_items = items_to_load;


      // });
    // }else if(_.isObject(workflows[1])){
      // var items_to_load = _.map(workflows, 'data');
      // proc.send({action: 'run', items: items_to_load});
      // child.running_items = items_to_load;
    // }




  });

};

EngineManager.prototype.broadcastResourcesInfo = function(){
  this.io.of('/engine/client')
    .emit('data', {event: 'resources', payload: this.resource_manager.to_json()});
  this.io.of('/engine/client')
    .emit('data', {event: 'resource_pool', payload: this.resource_pool_status});

};

EngineManager.prototype.broadcastEnginesInfo = function(){
  var payload = _.map(this.engine_processes, function(child, pid){
    var data = _.omit(child, 'process');
    return _.assign(data, {pid: pid});
  });

  this.io.of('/engine/client')
    .emit('data', {event: 'processes', payload: payload});

};

EngineManager.prototype._bindEvents = function(child){
  var proc = this.engine_processes[child.pid];

  var that = this;
  child.on('message', function(msg){

    var action = msg.action || msg.cmd;
    var result = null;
    proc.ee.emit(action, msg);

    switch(action) {
      case "status":
        var status = msg.status;
        logger.info('engine status to '+status);
        proc.status = status;
        that.emit(['child', child.pid, status].join('.'));


        var status_code = (status == 'started' ? 1 : 
                           (status == 'running' ? 2 : 
                            (status == 'stopped' ? 3 :
                             status == 'error' ? -1 : 0)));
        var status_msg = (status == 'error' ? msg.message : '');

        var event = {service_name: proc.name, status: status_code, message: status_msg};
        if(proc.name){
          var tp = "concert_workflow_engine_msgs/WorkflowsStatus";
          that.ros.publish('get_workflows_status', tp, event);
          }


        result = that.broadcastEnginesInfo();
        break;

      case "change_resource_ref_count":
        result = that.resource_manager.change_ref_count(msg.req_id, msg.delta);
        break;
      
      case 'ref_counted_release_resource':
        var ctx = msg.ctx;
        result = that.resource_manager.ref_counted_release(ctx.req_id);
        break;


      case 'allocate_resource':
        result = that.resource_manager.allocate(msg.key, msg.rapp, msg.uri, msg.remappings, msg.parameters, msg.options);
        break;
    
      case 'release_resource':
        result = that.resource_manager.release(msg.requester_id);
        break;

      case 'socket_broadcast':
        that.io.of(msg.namespace).emit(msg.key, msg.msg);
        result = true
        break;

      default:
        var action = msg.action;
        result = that.emit(['child', child.pid, action].join('.'), msg);
        break;
        
    }

    if(msg.__request_id){
      Promise.all([result]).then(function(results){
        var res = results[0];
        // child.send('return.'+msg.__request_id);
        // child.send({cmd: 'return', request_id: msg.__request_id, result: res}); // 'return.'+msg.__request_id);
        //
        //
        console.log("RESULT", results);

        child.send({cmd: 'return.'+msg.__request_id, request_id: msg.__request_id, result: res}); // 'return.'+msg.__request_id);

      });
    }

  });

};



EngineManager.prototype.killEngine = function(pid){
  logger.info('will kill engine : ' + pid);

  var that = this;
  var child = this.engine_processes[pid];
  var proc = child.process;
  // delete this.engine_processes[pid];

  child.ee.on('status', function(payload){
    console.log('---------------', payload);

    if(payload.status == 'stopped'){
      proc.kill('SIGTERM');
      delete that.engine_processes[pid];
      
      that.broadcastEnginesInfo();

    }
  });

  proc.send({cmd: 'stop'});




  this.broadcastEnginesInfo();

};


EngineManager.prototype.stopEngine = function(pid){
  var child = this.engine_processes[pid];

  child.on('message', function(msg){
    if(msg == 'engine_stopped'){
      child.kill('SIGTERM');
    }
  });

  delete this.engine_processes[pid];

};

EngineManager.prototype.restartEngine = function(pid){
  childEngine.on('message', function(msg){
    if(msg == 'engine_stopped'){
      childEngine.kill('SIGTERM');
      startEngine();
    }
  });
  childEngine.send({action: 'stop'});
};

module.exports = EngineManager;
