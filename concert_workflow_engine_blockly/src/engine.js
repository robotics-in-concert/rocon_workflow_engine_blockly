var _ = require('lodash'),
  R = require('ramda'),
  Promise = require('bluebird'),
  args = require('minimist')(process.argv.slice(2))
  EventEmitter2 = require('eventemitter2').EventEmitter2,
  ROSLIB = require('roslib'),
  Fiber = require('fibers')
  Future = require('fibers/future'),
  wait = Future.wait,
  Utils = require('./utils'),
  process_send2 = Utils.process_send2,
  util = require('util'),
  request = require('request'),
  Requester = require('./requester').Requester,
  Resource = require('./requester').Resource,
  UUID = require('node-uuid'),
  Ros = require('./ros'),
  URL = require('url');



/*
 * Engine class
 */

var Engine = function(opts){
  EventEmitter2.call(this, {wildcard: true});
  this.options = _.assign({
    ros_retries: 0,
    ros_retry_interval: 1000,
  }, opts);

  var engine = this;
  var that = this;

  var ros = engine.ros = new Ros(this.options);
  ros.on('status.**', function(){
    engine.emit(this.event);
  });


  this.executions = [];
  this.topics = [];
  this.my_dynamic_resource_ids = []; // dynamic resources id allocated in this engine.
  this.isWaitForTopicsTimeout = false;


  _.defer(function(){
    // engine.emit('started');
  });

};
util.inherits(Engine, EventEmitter2);

Engine.prototype.socketBroadcast = function(namespace, key, msg){
  process_send2({action: 'socket_broadcast', namespace:namespace, key: key, msg: msg})
  this.debug('socket broadcast', namespace, key, msg);
};


Engine.prototype.subscribe = function(topic, type, cb){
  return this.ros.subscribe(topic, type, cb);
};


Engine.prototype.publish = function(topic, type, msg){
  return this.pub(topic, type, msg);
};

Engine.prototype.pub = function(topic, type, msg){
  this.ros.publish(topic, type, msg);
  return null;
};



Engine.prototype.sleep = function(ms){
  var _sleep = function(ms){
    var future = new Future;
    setTimeout(function(){
      future.return();
    }, ms);
    return future;
  };
  _sleep(ms).wait();

};


Engine.prototype.getServiceParameter = function(service_name, key){
  var future = new Future();

  this.ros.getParam(['/services', service_name, key].join('/'))
    .then(function(result){
      future.return(result);
    });
  return future.wait();

};

Engine.prototype.runService = function(name, type, request){
  var e = this;

  var _runService = function(name, type, request){
    var future = new Future;

    var service = new ROSLIB.Service({
     ros : e.ros,
     name : name,
     servicetype : type
    });


    var request = new ROSLIB.ServiceRequest(request);

    service.callService(request, function(result){
      future.return(result);
    });
    return future;

  };
  return _runService(name, type, request).wait();
};

Engine.prototype.runCode = function(code){
  
  code = ["var f = Fiber(function(){ try{ ", code , " }catch(error_in_fiber){ console.log('error in fiber', error_in_fiber); throw error_in_fiber }}); f.run(); f"].join("\n");
  code = Utils.js_beautify(code);
  this.debug("---------------- scripts -----------------");
  this.debug(_.map(code.split(/\n/), function(line){ return line; }).join("\n"));
  this.debug("------------------------------------------");
  try{
    var f = eval(code);
    this.executions.push(f);
    this.log("scripts evaluated.");
  }catch(e){
    this.log('invalid block scripts. failed. - ' + e.toString());
    this.emit('status.error', e.message);
  }

};

// public - promise version
Engine.prototype.waitForTopicsReady = function(required_topics){
  return this.ros.waitForTopicsReady(required_topics);
};

// private - fiber version

Engine.prototype._waitForTopicsReadyF = function(required_topics){
  var engine = this;
  var fiber = Fiber.current;
  var old_remapped_topics_length = -1;

  var topic_wait_timeout = this.options.topic_wait_timeout;

  var wait_timeout_timer = null;
  var timer = setInterval(function(){
    if(!fiber.stopped){
      engine.ros.underlying.getTopics(function(topics){
        var remapped_topics = R.filter(function(t){ return R.contains(t, required_topics); })(topics);

        if (old_remapped_topics_length != remapped_topics.length){
          console.log('topic count check : ', [remapped_topics.length, required_topics.length].join("/"), remapped_topics);
          old_remapped_topics_length = remapped_topics.length;
        }

        if(remapped_topics.length >= required_topics.length){
          clearInterval(timer);
          if(!fiber.stopped){
            logger.info('all topic received: engine resume');
            engine.isWaitForTopicsTimeout = false;
            fiber.run();
            clearTimeout(wait_timeout_timer);
          }else{
            fiber.throwInto('stopped');
          }
        }
      });
    }else{
      clearInterval(timer);
      clearTimeout(wait_timeout_timer);
      console.log('running fiber will stop');
      fiber.throwInto('stopped');
    }
  }, 1000);

  wait_timeout_timer = setTimeout(function(){
    logger.info('wait timeout timer called: engine resume');
    engine.isWaitForTopicsTimeout = true;
    clearInterval(timer);
    fiber.run();
  }, topic_wait_timeout);

  Fiber.yield();
};


Engine.prototype.allocateResource = function(rapp, uri, remappings, parameters, options){

  var engine = this;
  var allocation_type = options.type || 'dynamic';

  var future = new Future();

  var key = null;
  if(allocation_type == 'static'){
    var key = rapp + uri;
  }
  process_send2({action: 'allocate_resource', key: key, rapp: rapp, uri: uri, remappings: remappings, parameters: parameters, options: options})
    .then(function(ctx){

      //if(ctx && allocation_type == 'dynamic')
      engine.my_dynamic_resource_ids.push(ctx.req_id);

      future.return(ctx);
    });

  return future.wait();
};

Engine.prototype.releaseResource = function(ctx){
  // process_send2({cmd: 'ref_counted_release_resource', ctx: ctx});
  return process_send2({cmd: 'release_resource', requester_id: ctx.req_id});
};

Engine.prototype._scheduled = function(rapp, uri, remappings, parameters, topics_count, name, callback){
  var engine = this;
  
  var r = new Requester(this);

  this.ros.underlying.getTopics(function(topics){
    engine.log("topics : ", topics);

    var res = new Resource();
    res.rapp = rapp;
    res.uri = uri;
    res.remappings = remappings;

    console.log("remapping ", res.remappings);

    res.parameters = parameters;

    r.send_allocation_request(res).then(function(reqId){

      var topics_ready = new Promise(function(resolve, reject){

        var timer = setInterval(function(){
          engine.ros.underlying.getTopics(function(topics){

            
            var remapped_topics = R.filter(R.match("^"+name))(topics);
            console.log('topic count check : ', remapped_topics.length);

            if(remapped_topics.length >= topics_count){
              clearInterval(timer);
              resolve();
            }
          });
        }, 1000);
      });
      topics_ready.then(function(){
        callback(r);
      });
    });
  });


};


Engine.prototype._changeResourceRefCount = function(rid, delta){
  delta = delta || 1;
  process_send2({cmd: 'change_resource_ref_count', req_id: rid, delta: delta});
};

Engine.prototype.incResourceRefCount = function(rid){
};

Engine.prototype.decResourceRefCount = function(rid){
};




Engine.prototype.runScheduledAction = function(ctx, name, type, goal, onResult, onFeedback, onTimeout, options){
  var name = _.detect(ctx.remappings, {remap_from: name}).remap_to;
  var engine = this;

  var required_topics = _.map(["feedback", "result", "status", "goal", "cancel"], function(suffix){ return name + "/" + suffix});
  engine._waitForTopicsReadyF(required_topics);
  
  this.ros.run_action(name, type, goal, 
    function(items){ onResult(items); },
    function(items){ onFeedback(items); },
    onTimeout,
    options,
    engine.isWaitForTopicsTimeout
  )
  
};

Engine.prototype.scheduledSubscribe = function(ctx, topic, type, callback){
  var name = _.detect(ctx.remappings, {remap_from: topic}).remap_to;
  this.ros.subscribe(name, type, callback);
  
};


Engine.prototype.scheduledPublish = function(ctx, topic, type, msg){
  var name = _.detect(ctx.remappings, {remap_from: topic}).remap_to;
  this.ros.publish(name, type, msg);

};

Engine.prototype.clear = function(){
  var that = this;

  // stop fibers
  this.executions.forEach(function(f){
    f.stopped = true;
  });
  this.executions = [];

  var proms = _.map(this.my_dynamic_resource_ids, function(rid){
    that.log('engine clear');
    return process_send2({cmd: 'release_resource', requester_id: rid});
  });
  
  return Promise.all(proms).then(function(){
    // that.unsubscribeAll();
    that.log('engine cleared');
  }).catch(function(e){
    that.log('fail - engine clear ' + e.toString());
    
  });;

};
Engine.prototype.print = function(msg){
  console.log(new Date().toString() + " - " + msg.green);

};

Engine.prototype.log = function(args){
  global.logger.info(args)
};

Engine.prototype.debug = function(args){
  global.logger.debug(args);
};

Engine.prototype.itemsToCode = function(items){
  var js = _.map(items, function(i){
    i = _.isString(i) ? JSON.parse(i) : i;
    return "// "+i.title+"\n"+i.js;
  }).join("\n\n");


  return js;

};

Engine.prototype.runItems = function(items) {
  var scripts = this.itemsToCode(items);
  this.runCode(scripts);
};



Engine.prototype.getParam = function(k, cb){
  var param = new ROSLIB.Param({
    ros : this.ros,
    name : k
  });
  param.get(cb)
};


Engine.prototype.setParam = function(k, v, cb){
  var paramClient = new ROSLIB.Service({
    ros : this.ros,
    name : '/rosapi/set_param',
    serviceType : 'rosapi/SetParam'
  });

  var request = new ROSLIB.ServiceRequest({
    name : k,
    value : JSON.stringify(v)
  });

  paramClient.callService(request, cb);
};

Engine.prototype.load = function(){
  this.runBlocks(null);

};
Engine.prototype.reload = function(){
  var that = this;
  this.clear();
  this.load();
};


Engine.prototype.createUI = function(name, meta){
  process_send2({cmd: 'create_ui', name: name, meta: meta});
};




module.exports = Engine;
